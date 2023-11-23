#include </mnt/Dados/caiopinho/carmen_lcad/pytorch_cpp/libtorch/include/torch/csrc/api/include/torch/torch.h>
#include <iostream>
#include <vector>
#include </mnt/Dados/caiopinho/carmen_lcad/pytorch_cpp/libtorch/include/torch/script.h>

/*int main() {
  torch::Tensor tensor = torch::rand({2, 3});
  std::cout << tensor << std::endl;
}*/
int main() {
    // Load the Torch Script module
    auto module = torch::jit::load("/mnt/Dados/caiopinho/carmen_lcad/model_clean.pt");

    // Create a tensor with specific values
    //torch::Tensor t = torch::tensor({{1.26659207, -0.27974198, -1.24881179, -2.58876423, 1.27859044, -0.25543732, -1.24577649, -2.4471998, 0.05157178}});
    //torch::Tensor t = torch::tensor({{7757750.975, -363847.325, -2.485, 2.5, 7757759.112, -363841.075, -2.485, 2.902, 0.0}});


// Assuming that 'x' is a vector of input features
std::vector<float> x = {7757750.975, -363847.325, -2.485, 2.5, 7757759.112, -363841.075, -2.485, 2.902, 0.0};

// The mean and standard deviation of each feature
std::vector<float> mean = {7757219.02030919, -363790.45257479, 0.02507255, 7.59847813, 7757221.91891166, -363789.01252881, 0.02444362, 7.47429562, -0.00269055};
std::vector<float> std_dev = {419.98896269, 203.30314877, 2.00996865, 1.96946407, 420.14477201, 203.81701318, 2.01436103, 1.86837855, 0.05217095};

// Scale the input features
for (size_t i = 0; i < x.size(); ++i) {
    x[i] = (x[i] - mean[i]) / std_dev[i];
}
torch::Tensor t = torch::tensor({{x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8]}});


    // Create a vector to hold the inputs to the model
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(t);
    //inputs.push_back(torch::ones({1, 9}));  // Adjust as necessary

    // Forward pass
    at::Tensor output = module.forward(inputs).toTensor();

    // Print the output
    std::cout << output << std::endl;

    return 0;
}