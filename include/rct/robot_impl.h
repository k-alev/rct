// #include <robot.h>
// #include <force_torque_sensor_interface.h>

// namespace rct
// {
// template <>
// void robot::read_from_robot<ForceTorqueSensorHandle>(const ForceTorqueSensorHandle &handle)
// {
//   const double *tmp_frc, *tmp_trq;

//   tmp_frc = handle.getForce();
//   tmp_trq = handle.getTorque();

//   for (unsigned int i = 0; i < 3; i++)
//   {
//     ft.force(i) = tmp_frc[i];
//     ft.torque(i) = tmp_trq[i];
//   }

//   std::cout << "Reading HI " << this->ft.force(2) << std::endl;
// }
// } // namespace rct

// template <class T>
// void robot::read_HI(T handle)
// {
//     // ft.force.data = handle.getForce();
// 	// ft.torque.data = handle.getTorque();
//     std::cout<<"Reading HI"<<std::endl;
// }

// }