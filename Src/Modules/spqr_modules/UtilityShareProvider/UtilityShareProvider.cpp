#include "UtilityShareProvider.h"

#include <unistd.h>
#include <iostream>


UtilityShareProvider::UtilityShareProvider(){
    SPQR::ConfigurationParameters();
}




void UtilityShareProvider::update(UtilityShare& us) {

    unsigned k;


    std::vector<int> utility_vector = theRole.utility_vector;

    for(k = 1; k < utility_vector.size(); k++){
        //std::cout<<"utility at "<<k<<" = "<<utility_vector.at(k)<<std::endl;
        switch(k){
            case 1: us.striker = utility_vector.at(k); break;

            case 2: us.defender = utility_vector.at(k); break;

            case 3: us.supporter = utility_vector.at(k); break;

            case 4: us.jolly = utility_vector.at(k); break;
        }


    }
    us.striker = 3;

    return;
}

MAKE_MODULE(UtilityShareProvider, modeling)
