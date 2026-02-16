
void all_blind_random() {
    // RUN BLIND RANDOM SEARCH
    int amnt_iter = 10;
    std::vector<std::string> datasets = {f10, f20, f50, f100, r10, r20, r50, r100};
    long long best;
    long double avg;
    double avg_runtime;
    Instance instance;
    Solution initial;

    for (std::string dataset : datasets) {
        best = INF;
        avg = 0;
        avg_runtime = 0;

        for (int i = 0; i < amnt_iter; i++) {
            auto start = std::chrono::high_resolution_clock::now();
            instance = read_instance(dataset);
            initial = simple_initial_solution(instance.n);
            
            Solution best_random = blind_random_search(
                instance,
                initial,
                calculate_total_waiting_time
            );
            auto stop = std::chrono::high_resolution_clock::now();
            long long final_val = calculate_total_waiting_time(instance, best_random);

            avg += final_val;
            best = std::min(final_val, best);
            
            std::chrono::duration<double> duration = stop - start;
            avg_runtime += duration.count();
        }

        avg_runtime = avg_runtime / amnt_iter;
        avg = avg / amnt_iter;
        long long initial_objective = calculate_total_waiting_time(instance, initial);
        double improvement_percent = 100 * (initial_objective - best) / static_cast<double>(initial_objective);


        bool result = save_to_csv("Random Search", dataset, avg, best, improvement_percent, avg_runtime);
        if (!result) {
            std::cout << "Error..." << "\n";
        }
    }
}