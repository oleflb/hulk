use super::*;

pub(super) fn solve_problem(problem: &Problem) -> Option<GlobalLocalizationResult> {
    let CandidateSearch {
        mut candidates,
        truncated,
        ..
    } = candidate_hypotheses(problem);
    candidates.sort_by(compare_candidates);
    let unique_candidates = remove_equivalent_candidates(candidates, &problem.map);
    let accepted_candidates = unique_candidates
        .into_iter()
        .filter(|candidate| passes_basic_acceptance(candidate, problem.cfg))
        .collect::<Vec<_>>();
    accepted_candidates.first()?;
    if problem.detections_truncated || truncated {
        return None;
    }

    let stable = stable_candidate(problem, &accepted_candidates)?;
    let stable = oriented_candidate(problem, &stable);
    let associations = to_public(&stable, problem);
    if !robot_position_within_field_boundary(problem, associations.robot_to_field) {
        return None;
    }

    Some(GlobalLocalizationResult::UniqueModuloSymmetry(associations))
}

fn candidate_hypotheses(problem: &Problem) -> CandidateSearch {
    let mut search = CandidateSearch {
        candidates: Vec::new(),
        truncated: false,
    };
    let mut cheap = cheap_triplet_seeds(problem);
    cheap.candidates.sort_by(compare_cheap_candidates);
    let cheap_candidates = remove_equivalent_cheap_candidates(cheap.candidates, &problem.map);
    search.truncated |= cheap.truncated;

    let mut seeds = seed_states(problem, cheap_candidates);
    let mut refinements = 0;
    let mut queue = seed_queue(&seeds);

    while refinements < MAX_REFINED_CANDIDATES {
        let Some(entry) = pop_current_seed(&mut queue, &seeds) else {
            break;
        };
        refine_seed(problem, &mut search, &mut seeds, entry.index);
        refinements += 1;
    }
    if seeds
        .iter()
        .any(|seed| !seed.refined && seed.upper_bound.is_finite())
    {
        search.truncated = true;
    }

    search
}

fn remove_equivalent_cheap_candidates(
    candidates: Vec<CheapCandidate>,
    map: &LandmarkMap,
) -> Vec<CheapCandidate> {
    let mut unique = Vec::new();
    let mut keys = HashSet::new();
    for candidate in candidates {
        if !keys.insert(canonical_cheap_candidate_key(&candidate, map)) {
            continue;
        }
        unique.push(candidate);
    }
    unique
}

pub(super) fn canonical_cheap_candidate_key(
    candidate: &CheapCandidate,
    map: &LandmarkMap,
) -> AssociationKey {
    candidate
        .key
        .min(symmetric_cheap_candidate_key(candidate, map))
}

fn symmetric_cheap_candidate_key(candidate: &CheapCandidate, map: &LandmarkMap) -> AssociationKey {
    candidate.key.symmetric(map)
}

fn seed_states(problem: &Problem, cheap_candidates: Vec<CheapCandidate>) -> Vec<SeedState> {
    cheap_candidates
        .into_iter()
        .map(|seed| {
            let upper_bound = optimistic_seed_score_fast(problem, seed.transform);
            SeedState {
                upper_bound,
                seed,
                refined: false,
            }
        })
        .collect()
}

fn seed_queue(seeds: &[SeedState]) -> BinaryHeap<SeedQueueEntry> {
    let mut queue = BinaryHeap::new();
    for index in 0..seeds.len() {
        push_seed_entry(&mut queue, seeds, index);
    }
    queue
}

fn push_seed_entry(queue: &mut BinaryHeap<SeedQueueEntry>, seeds: &[SeedState], index: usize) {
    if seeds[index].refined {
        return;
    }
    if let Ok(upper_bound) = NotNan::new(seeds[index].upper_bound) {
        queue.push(SeedQueueEntry { upper_bound, index });
    }
}

fn pop_current_seed(
    queue: &mut BinaryHeap<SeedQueueEntry>,
    seeds: &[SeedState],
) -> Option<SeedQueueEntry> {
    while let Some(entry) = queue.pop() {
        let seed = seeds.get(entry.index)?;
        if !seed.refined {
            return Some(entry);
        }
    }
    None
}

fn refine_seed(
    problem: &Problem,
    search: &mut CandidateSearch,
    seeds: &mut [SeedState],
    index: usize,
) {
    seeds[index].refined = true;
    if let Some(candidate) = build_fitted_candidate(problem, seeds[index].seed.transform) {
        insert_candidate(search, candidate);
    }
}

fn insert_candidate(search: &mut CandidateSearch, candidate: Candidate) {
    if search.candidates.len() < MAX_REFINED_CANDIDATES {
        search.candidates.push(candidate);
        return;
    }

    search.truncated = true;
    let worst_index = search
        .candidates
        .iter()
        .enumerate()
        .max_by(|(_, left), (_, right)| compare_candidates(left, right))
        .map(|(index, _)| index);
    if let Some(worst_index) = worst_index
        && compare_candidates(&candidate, &search.candidates[worst_index]).is_lt()
    {
        search.candidates[worst_index] = candidate;
    }
}
