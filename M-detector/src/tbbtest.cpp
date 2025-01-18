#include <iostream>
 #include <vector>
 #include <random>
 #include <algorithm>
  #include <chrono>
  #include <execution>
  
 void printDuration(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end, const char *message) {
          auto diff = end - start;
         std::cout << message << ' ' << std::chrono::duration <double, std::milli> (diff).count() << " ms\n";
 }
 
 template<typename T>
 void test(const T &policy, const std::vector<double> &data, const int repeat, const char *message) {
     for(int i = 0; i < repeat; ++i) {
         std::vector<double> curr_data(data);
 
         const auto start = std::chrono::steady_clock::now();
         std::sort(policy, curr_data.begin(), curr_data.end());
         const auto end = std::chrono::steady_clock::now();
         printDuration(start, end, message);
    }
     std::cout << '\n';
 }
 
 int main() {
     // Test samples and repeat factor
     constexpr size_t samples{5'000'000};
     constexpr int repeat{10};
 
     // Fill a vector with samples numbers
     std::random_device rd;
     std::mt19937_64 mre(rd());
     std::uniform_real_distribution<double> urd(0.0, 1.0);
 
     std::vector<double> data(samples);
     for(auto &e : data) {
         e = urd(mre);
     }
 
     // Sort data using different execution policies
    //  std::cout << "std::execution::seq\n";
    //  test(std::execution::seq, data, repeat, "Elapsed time");
 
    //  std::cout << "std::execution::par\n";
    //  test(std::execution::par, data, repeat, "Elapsed time");

    std::vector<int> index(10000);
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }
    std::vector<int> out;
    out.reserve(1000000);
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const int &i)
    {   
        if(i == 10)
        {
            return;
        }
        out[i] = 1;
    });
    int sum = 0;
    for (int i = 0; i < 10000; i++)
    {
        sum +=out[i];
    }
    std::cout << "sum: " << sum << std::endl;
 }