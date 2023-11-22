#include </mnt/Dados/caiopinho/carmen_lcad/pytorch_cpp/libtorch/include/torch/csrc/api/include/torch/torch.h>
#include <iostream>

int main() {
  torch::Tensor tensor = torch::rand({2, 3});
  std::cout << tensor << std::endl;
}