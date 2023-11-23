#include </mnt/Dados/caiopinho/carmen_lcad/pytorch_cpp/libtorch/include/torch/csrc/api/include/torch/torch.h>
#include <iostream>
#include </mnt/Dados/caiopinho/carmen_lcad/pytorch_cpp/libtorch/include/torch/script.h>

/*int main() {
  torch::Tensor tensor = torch::rand({2, 3});
  std::cout << tensor << std::endl;
}*/
int main() {
    // Load the Torch Script module
    auto module = torch::jit::load("/mnt/Dados/caiopinho/carmen_lcad/model_clean.pt");

    // Create a tensor with specific values
    //torch::Tensor t = torch::tensor({{0.2144,  1.4291, -1.2174, -0.6024,  0.2654,  1.4626,  1.5191, -0.3682, 1.2702}});
    torch::Tensor t = torch::tensor({{1.26659207, -0.27974198, -1.24881179, -2.58876423, 1.27859044, -0.25543732, -1.24577649, -2.4471998, 0.05157178}});
    //torch::Tensor t = torch::tensor({{7757706.153, -363881.716, -2.482, 8.499, 7757721.272, -363870.038, -2.487, 4.803, 0.001}});
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