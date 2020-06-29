#ifndef RANDOM_NUM_GENERATOR_HPP
#define RANDOM_NUM_GENERATOR_HPP

#include <random>
#include <vector>

class random_num_generator {
public:
    random_num_generator();
    void initialize_uniform_int_distributor(int lower_bound, int heigher_bound);
    void initialize_uniform_double_distributor(double lower_bound, double heigher_bound);
    void initialize_normal_distributibutor(double mean, double stddev);
    int generate_int();
    void generate_int(std::vector<int>& vec, int num);
    double generate_double();
    double generate_double_with_normal_distribution();
    void generate_double(std::vector<double>& vec, int num);
    void generate_double_with_normal_distribution(std::vector<double>& vec, int num);
private:
    std::uniform_int_distribution<int> m_int_distribution;
    std::uniform_real_distribution<double> m_real_distribution;
    std::normal_distribution<double> m_normal_distribution;
    std::default_random_engine m_engine;
};
#endif // RANDOM_NUM_GENERATOR_HPP
