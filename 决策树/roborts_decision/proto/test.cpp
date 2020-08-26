#include <iostream>
#include "decision.pb.h"
#include "io.h"

int main(){
    roborts_decision::Referee ref;
    roborts_common::ReadProtoFromTextFile("../config/topic_name.prototxt", &ref);
    std::cout <<"***"<< ref.game_result() << "**"<< std::endl;

    roborts_decision::DecisionConfig decision_config;
    return 0;
}