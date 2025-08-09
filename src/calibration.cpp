#include "parse_dataset.h"
#include "utils.h"
#include <iostream>



int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " dataset.txt\n";
        return 1;
    }
    Dataset ds;
    if (!loadDataset(argv[1], ds)) {
        std::cerr << "Failed to read dataset: " << argv[1] << "\n";
        return 1;
    }

    print_infos(ds);



    return 0;
}