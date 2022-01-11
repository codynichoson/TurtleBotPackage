#include <iostream>

struct Transform{
    int theta;
    int x;
    int y;
}
Tab, Tbc;

int main(){
    std::cout << "Enter transform T_{a,b}: \n";
    std::cout << "deg: ", std::cin >> Tab.theta;
    std::cout << "x: ", std::cin >> Tab.x;
    std::cout << "y: ", std::cin >> Tab.y;

    std::cout << "Enter transform T_{b,c}: \n";
    std::cout << "deg: ", std::cin >> Tbc.theta;
    std::cout << "x: ", std::cin >> Tbc.x;
    std::cout << "y: ", std::cin >> Tbc.y;
    
    // Compute Tab, Tba, Tbc, Tcb, Tac, and Tca

    // Display Tab, Tba, Tbc, Tcb, Tac, and Tca

}
