use factrs::linalg::Numeric;

use super::LandmarkAssociationConfig;

/// Runs log-domain Sinkhorn with one dustbin row and one dustbin column.
/// Returns only real detection-to-candidate log assignments, flattened in
/// row-major order.
pub(super) fn rectangular_log_sinkhorn_log_assignments<T: Numeric>(
    config: &LandmarkAssociationConfig,
    costs: &[T],
    num_detections: usize,
    num_candidates: usize,
) -> Vec<T> {
    debug_assert_eq!(costs.len(), num_detections * num_candidates);
    assert!(
        config.temperature > 0.0,
        "sinkhorn temperature must be positive"
    );

    if num_candidates == 0 || num_detections == 0 {
        return Vec::new();
    }

    let num_rows = num_detections + 1;
    let num_cols = num_candidates + 1;
    let temperature = T::from(config.temperature);
    let unmatched_landmark_cost = T::from(config.unmatched_landmark_cost);
    let unmatched_detection_cost = T::from(config.unmatched_detection_cost);
    let log_unmatched_landmark_mass = T::from(num_candidates as f64).ln();
    let log_unmatched_detection_mass = T::from(num_detections as f64).ln();

    let log_kernel = (0..num_rows)
        .flat_map(|row| {
            (0..num_cols).map(move |col| {
                let cost = if row < num_detections && col < num_candidates {
                    costs[row * num_candidates + col]
                } else if row == num_detections && col < num_candidates {
                    unmatched_landmark_cost
                } else if row < num_detections && col == num_candidates {
                    unmatched_detection_cost
                } else {
                    T::zero()
                };

                -cost / temperature
            })
        })
        .collect::<Vec<_>>();

    let mut log_u = vec![T::zero(); num_rows];
    let mut log_v = vec![T::zero(); num_cols];

    for _ in 0..config.sinkhorn_iterations {
        for (row, log_u_row) in log_u.iter_mut().enumerate() {
            let log_row_mass = if row < num_detections {
                T::zero() // log(1)
            } else {
                // The dustbin row absorbs any number of unmatched landmarks.
                log_unmatched_landmark_mass
            };

            *log_u_row = log_row_mass - log_sum_exp_row(&log_kernel, &log_v, row, num_cols);
        }

        for (col, log_v_col) in log_v.iter_mut().enumerate() {
            let log_col_mass = if col < num_candidates {
                T::zero() // log(1)
            } else {
                // The dustbin column absorbs any number of unmatched detections.
                log_unmatched_detection_mass
            };

            *log_v_col =
                log_col_mass - log_sum_exp_col(&log_kernel, &log_u, col, num_rows, num_cols);
        }
    }

    (0..num_detections)
        .flat_map(|row| {
            let log_kernel = &log_kernel;
            let log_u = &log_u;
            let log_v = &log_v;

            (0..num_candidates)
                .map(move |col| log_kernel[row * num_cols + col] + log_u[row] + log_v[col])
        })
        .collect()
}

fn log_sum_exp_row<T: Numeric>(log_kernel: &[T], log_v: &[T], row: usize, num_cols: usize) -> T {
    debug_assert!(num_cols > 0);
    debug_assert_eq!(log_v.len(), num_cols);

    let row_offset = row * num_cols;
    let max_value = (0..num_cols)
        .map(|col| log_kernel[row_offset + col] + log_v[col])
        .reduce(T::max)
        .expect("rows must have at least one column");

    let sum = (0..num_cols)
        .map(|col| (log_kernel[row_offset + col] + log_v[col] - max_value).exp())
        .fold(T::zero(), |sum, value| sum + value);

    max_value + sum.ln()
}

fn log_sum_exp_col<T: Numeric>(
    log_kernel: &[T],
    log_u: &[T],
    col: usize,
    num_rows: usize,
    num_cols: usize,
) -> T {
    debug_assert!(num_rows > 0);
    debug_assert_eq!(log_u.len(), num_rows);

    let max_value = (0..num_rows)
        .map(|row| log_kernel[row * num_cols + col] + log_u[row])
        .reduce(T::max)
        .expect("columns must have at least one row");

    let sum = (0..num_rows)
        .map(|row| (log_kernel[row * num_cols + col] + log_u[row] - max_value).exp())
        .fold(T::zero(), |sum, value| sum + value);

    max_value + sum.ln()
}

#[cfg(test)]
mod tests {
    use factrs::linalg::Numeric;

    use super::*;

    fn rectangular_log_sinkhorn<T: Numeric>(
        config: &LandmarkAssociationConfig,
        costs: &[T],
        num_detections: usize,
        num_candidates: usize,
    ) -> Vec<T> {
        rectangular_log_sinkhorn_log_assignments(config, costs, num_detections, num_candidates)
            .into_iter()
            .map(|log_assignment| log_assignment.exp())
            .collect()
    }

    fn sinkhorn_config() -> LandmarkAssociationConfig {
        LandmarkAssociationConfig {
            sinkhorn_iterations: 100,
            temperature: 0.1,
            ..LandmarkAssociationConfig::default()
        }
    }

    #[test]
    fn sinkhorn_prefers_low_cost_diagonal_assignments() {
        let assignments =
            rectangular_log_sinkhorn(&sinkhorn_config(), &[0.0, 10.0, 10.0, 0.0], 2, 2);

        assert!(assignments[0] > assignments[1]);
        assert!(assignments[3] > assignments[2]);
    }

    #[test]
    fn sinkhorn_dustbins_absorb_unmatched_candidates() {
        let assignments = rectangular_log_sinkhorn(&sinkhorn_config(), &[0.0, 100.0], 1, 2);

        assert!(assignments[0] > assignments[1]);
        assert!(assignments[1] < 1e-9);
    }

    #[test]
    fn sparse_detection_does_not_match_all_candidates() {
        let config = LandmarkAssociationConfig {
            unmatched_landmark_cost: 0.0,
            unmatched_detection_cost: 25.0,
            ..LandmarkAssociationConfig::default()
        };

        let assignments = rectangular_log_sinkhorn(&config, &[0.0, 100.0, 100.0, 100.0], 1, 4);

        assert!(assignments[0] > 0.8);
        assert!(assignments[1] < 1.0e-5);
        assert!(assignments[2] < 1.0e-5);
        assert!(assignments[3] < 1.0e-5);
        assert!(assignments.iter().sum::<f64>() <= 1.0);
    }

    #[test]
    fn expensive_unmatched_landmarks_overconstrain_sparse_detection() {
        let config = LandmarkAssociationConfig {
            unmatched_landmark_cost: 1000.0,
            unmatched_detection_cost: 1000.0,
            ..LandmarkAssociationConfig::default()
        };

        let assignments = rectangular_log_sinkhorn(&config, &[0.0, 100.0, 100.0, 100.0], 1, 4);

        assert!(assignments.iter().all(|assignment| *assignment > 0.99));
    }

    #[test]
    fn sinkhorn_dustbins_absorb_unmatched_detections() {
        let assignments = rectangular_log_sinkhorn(&sinkhorn_config(), &[0.0, 100.0], 2, 1);

        assert!(assignments[0] > assignments[1]);
        assert!(assignments[1] < 1e-9);
    }

    #[test]
    fn sinkhorn_stays_finite_for_large_equal_costs() {
        let log_assignments = rectangular_log_sinkhorn_log_assignments(
            &sinkhorn_config(),
            &[10_000.0, 10_000.0, 10_000.0, 10_000.0],
            2,
            2,
        );

        for log_assignment in &log_assignments {
            assert!(log_assignment.is_finite());
        }
        assert!((log_assignments[0] - log_assignments[1]).abs() < 1e-9);
        assert!((log_assignments[0] - log_assignments[2]).abs() < 1e-9);
        assert!((log_assignments[0] - log_assignments[3]).abs() < 1e-9);
    }
}
