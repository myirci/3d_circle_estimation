#include "random_num_generator.hpp"

random_num_generator::random_num_generator() {
    m_int_distribution = std::uniform_int_distribution<int>(1,100);
    m_real_distribution = std::uniform_real_distribution<double>(0.0, 1.0);
    m_normal_distribution = std::normal_distribution<double>(0.0, 1.0);
    m_engine =  std::default_random_engine(std::random_device()());
}

void random_num_generator::initialize_uniform_int_distributor(int lower_bound, int heigher_bound) {
    m_int_distribution = std::uniform_int_distribution<int>(lower_bound, heigher_bound);
}

void random_num_generator::initialize_uniform_double_distributor(double lower_bound, double heigher_bound) {
    m_real_distribution = std::uniform_real_distribution<double>(lower_bound, heigher_bound);
}

void random_num_generator::initialize_normal_distributibutor(double mean, double stddev) {
    m_normal_distribution = std::normal_distribution<double>(mean, stddev);
}

int random_num_generator::generate_int() {
    return m_int_distribution(m_engine);
}

double random_num_generator::generate_double() {
    return m_real_distribution(m_engine);
}

double random_num_generator::generate_double_with_normal_distribution() {
    return m_normal_distribution(m_engine);
}

void random_num_generator::generate_int(std::vector<int>& vec, int num) {
    for(int i = 0; i < num; ++i) {
        vec.push_back(m_int_distribution(m_engine));
    }
}

void random_num_generator::generate_double(std::vector<double>& vec, int num) {
    for(int i = 0; i < num; ++i) {
        vec.push_back(m_real_distribution(m_engine));
    }
}

void random_num_generator::generate_double_with_normal_distribution(std::vector<double>& vec, int num) {
    for(int i = 0; i < num; ++i) {
        vec.push_back(m_normal_distribution(m_engine));
    }
}
