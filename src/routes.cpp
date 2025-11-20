#include "routes.h"

namespace Routes {
    void blueGoalSide() {

        std::cout << "Blue Goal Side" << std::endl;
        Chassis::getChassis().moveToPoint(0, 24, 1000000, {}, false);

    }

    void blueRingSide() {

        std::cout << "Blue Ring Side" << std::endl;
        Chassis::getChassis().moveToPoint(0, 24, 1000000, {}, false);
        
    }

    void redGoalSide() {

        std::cout << "Red Goal Side" << std::endl;
        Chassis::getChassis().moveToPoint(0, 24, 1000000, {}, false);
        
    }

    void redRingSide() {

        std::cout << "Red Ring Side" << std::endl;
        Chassis::getChassis().moveToPoint(0, 24, 1000000, {}, false);
        
    }

    void skillsAuton() {
        
        std::cout << "Skills Auton" << std::endl;
        Chassis::getChassis().moveToPoint(0, 24, 1000000, {}, false);

    }
}