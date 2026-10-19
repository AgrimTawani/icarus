#include "../../../simulation/plugins/BatteryLoadModel.hh"
#include <cassert>
#include <limits>
int main() {
 using icarus::RotorBatteryPower;
 assert(RotorBatteryPower(45,700,100,{0,0,0,0}) == 45);
 assert(RotorBatteryPower(45,700,100,{100,-100,100,-100}) == 700);
 assert(RotorBatteryPower(45,700,100,{50,50}) == 45+655*.125);
 assert(RotorBatteryPower(45,700,100,{200}) == 45+655*8);
 assert(RotorBatteryPower(45,700,100,{}) == 700);
 assert(RotorBatteryPower(45,700,100,{std::numeric_limits<double>::quiet_NaN()}) == 700);
 assert(RotorBatteryPower(45,700,100,{std::numeric_limits<double>::infinity()}) == 700);
 assert(RotorBatteryPower(45,700,100,{1e300}) == 700);
 bool rejected=false;try{RotorBatteryPower(45,20,100,{0});}catch(const std::invalid_argument&){rejected=true;}
 assert(rejected);
 rejected=false;try{RotorBatteryPower(45,700,0,{0});}catch(const std::invalid_argument&){rejected=true;}
 assert(rejected);
}
