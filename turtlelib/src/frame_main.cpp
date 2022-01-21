#include <iostream>
#include <string>
#include <cmath>
#include "turtlelib/rigid2d.hpp"

int main(){

    turtlelib::Transform2D Tab, Tbc, Tba, Tcb, x;
    turtlelib::Vector2D v_b;
    turtlelib::Twist2D V_b;

    std::cout << "Enter transform T_{a,b}: \n";
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}: \n";
    std::cin >> Tbc;
    std::cout << "T_{a,b}: " << Tab; 
    Tba = Tab.inv();
    std::cout << "T_{b,a}: " << Tba; 
    std::cout << "T_{b,c}: " << Tbc;
    Tcb = Tbc.inv(); 
    std::cout << "T_{c,b}: " << Tcb; 
    std::cout << "T_{a,c}: " << Tab*Tbc;
    std::cout << "T_{c,a}: " << (Tab*Tbc).inv();

    std::cout << "Enter vector v_b: \n";
    std::cin >> v_b;
    std::cout << "v_bhat: " << normalize(v_b);
    std::cout << "v_a: " << x.convertVec(Tab, v_b);
    std::cout << "v_b: " << v_b;
    std::cout << "v_c: " << x.convertVec(Tcb, v_b);
    
    std::cout << "Enter twist V_b: \n";
    std::cin >> V_b;
    std::cout << "V_a " << x.convertTwist(Tab, V_b);
    std::cout << "V_b " << V_b;
    std::cout << "V_c " << x.convertTwist(Tcb, V_b);

}