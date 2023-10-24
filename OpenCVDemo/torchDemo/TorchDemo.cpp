#include <torch/torch.h> 
#include <iostream>
#include "../include/network.h"


using namespace torch;
void test(){
    Net network(50,50);
    std::cout << network << std::endl;
    Tensor x, output;
    x = torch::randn({2,50});
    output = network->forward(x);
}