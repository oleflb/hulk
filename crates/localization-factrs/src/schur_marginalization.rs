use std::{
    collections::{HashMap, HashSet},
    ops::Mul,
};

use crate::{prior_factor::SchurPriorResidual, symbols::State};
use factrs::{
    containers::{FactorBuilder, Key, ValuesOrder},
    core::{GaussNewton, Graph, Values},
    residuals::DynVarPack,
    traits::Optimizer,
    variables::{SE23, VariableSafe},
};
use faer::{
    Mat, Par,
    linalg::solvers::Solve,
    sparse::{SparseColMat, linalg::matmul::sparse_sparse_matmul},
};
use faer_ext::IntoNalgebra;

pub fn marginalize(optimizer: &mut GaussNewton, values: &mut Values, cutoff_state: State) {
    let keys_to_marginalize = find_keys_to_marginalize(values, cutoff_state);
    if keys_to_marginalize.is_empty() {
        return;
    }

    let keys_to_marginalize_set: HashSet<_> = keys_to_marginalize.iter().copied().collect();

    // Remove only the factors connected to a marginalized state
    let removed_factors = optimizer.graph_mut().remove_factors(|factor| {
        factor
            .keys()
            .iter()
            .any(|key| keys_to_marginalize_set.contains(key))
    });

    let boundary_keys = find_markov_blanket(&keys_to_marginalize_set, &removed_factors);

    let mut temporary_graph = Graph::default();
    for factor in removed_factors {
        temporary_graph.add_factor(factor);
    }

    let mut local_keys = Vec::new();
    for k in &keys_to_marginalize {
        local_keys.push(*k);
    }
    let keep_offset_keys = local_keys.len();
    for k in &boundary_keys {
        local_keys.push(*k);
    }

    let mut map = HashMap::default();
    let mut offset = 0usize;
    let mut keep_offset_dim = 0usize;

    for (index, key) in local_keys.iter().enumerate() {
        let dim = values.get_raw(*key).expect("missing key").dim();
        map.insert(*key, factrs::containers::Idx { idx: offset, dim });
        offset += dim;

        if index + 1 == keep_offset_keys {
            keep_offset_dim = offset;
        }
    }

    log::debug!("marginalizing {keep_offset_dim} DOFs");
    let value_order = ValuesOrder::new(map);
    let linearized_graph = temporary_graph.linearize(values);
    let graph_order = linearized_graph.sparsity_pattern(value_order);
    let linearized_graph = linearized_graph.with_order(&graph_order);

    let residual_jacobian = linearized_graph.residual_jacobian();
    let linear_system = from_jacobian(residual_jacobian.diff, residual_jacobian.value);

    let h_dense = linear_system.h.to_dense();
    let b = linear_system.b;

    let (h_mm, h_mk, h_km, h_kk) = h_dense.split_at(keep_offset_dim, keep_offset_dim);
    let (b_m, b_k) = b.split_at_row(keep_offset_dim);

    let lu = h_mm.partial_piv_lu();
    let solved_h = lu.solve(&h_mk);
    let solved_b = lu.solve(&b_m);

    let h_prior = h_kk - h_km * solved_h;
    let b_prior = b_k - h_km * solved_b;

    values.retain(|key| !keys_to_marginalize_set.contains(key));

    if boundary_keys.is_empty() {
        return;
    }

    let h_prior_nalgebra = h_prior.as_ref().into_nalgebra().clone_owned();
    let b_prior_nalgebra = b_prior.as_ref().into_nalgebra().column(0).clone_owned();

    let mut h_sym = (&h_prior_nalgebra + &h_prior_nalgebra.transpose()) * 0.5;
    let mut jitter = 1.0e-9;

    let cholesky = loop {
        if let Some(cholesky) = h_sym.clone().cholesky() {
            break cholesky;
        }

        assert!(
            jitter <= 1.0,
            "information matrix must be positive definite"
        );

        h_sym += nalgebra::DMatrix::identity(h_sym.nrows(), h_sym.ncols()) * jitter;
        jitter *= 10.0;
    };

    let lower_triangular = cholesky.l();
    let jacobian_matrix = lower_triangular.transpose();

    let target_error_vector = lower_triangular
        .solve_lower_triangular(&b_prior_nalgebra)
        .expect("failed to solve for target error");

    let input = DynVarPack::new(boundary_keys.clone()).expect("boundary keys must be unique");
    let factor = FactorBuilder::new_dyn(
        SchurPriorResidual::new(
            boundary_keys.clone(),
            jacobian_matrix,
            target_error_vector,
            make_linearization_point(values, &boundary_keys),
        ),
        input,
    )
    .build();

    optimizer.graph_mut().add_factor(factor);
}

fn find_markov_blanket(
    keys_to_marginalize_set: &HashSet<Key>,
    removed_factors: &[factrs::core::Factor],
) -> Vec<Key> {
    let mut boundary_keys = removed_factors
        .iter()
        .flat_map(|factor| {
            factor
                .keys()
                .iter()
                .filter(|key| !keys_to_marginalize_set.contains(key))
        })
        .copied()
        .collect::<Vec<_>>();

    let mut unique_boundary_key = HashSet::new();
    boundary_keys.retain(|key| unique_boundary_key.insert(*key));
    boundary_keys
}

fn find_keys_to_marginalize(values: &Values, cutoff_key: State) -> Vec<Key> {
    let cutoff_key: Key = cutoff_key.into();

    values
        .iter()
        .filter(|(key, value)| value.is::<SE23>() && key.0 < cutoff_key.0)
        .map(|(key, _)| *key)
        .collect()
}

fn make_linearization_point(values: &Values, keys: &[Key]) -> Vec<Box<dyn VariableSafe>> {
    keys.iter()
        .map(|k| {
            values
                .get_raw(*k)
                .unwrap_or_else(|| panic!("missing key {:?}", k))
                .clone_box()
        })
        .collect()
}

pub struct LinearSystem {
    h: SparseColMat<usize, f64>,
    b: Mat<f64>,
}

pub fn from_jacobian(jacobian: SparseColMat<usize, f64>, residual: Mat<f64>) -> LinearSystem {
    let jacobian_transpose = jacobian.transpose().to_col_major().unwrap();

    let hessian_matrix = sparse_sparse_matmul(
        jacobian_transpose.as_ref(),
        jacobian.as_ref(),
        1.0,
        Par::Seq,
    )
    .unwrap();

    let information_vector = jacobian_transpose.mul(residual);

    LinearSystem {
        h: hessian_matrix,
        b: information_vector,
    }
}
