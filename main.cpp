#include "adcp_measurement_model.hpp"

int main()
{
    boost::array<double,200> X_EKF;
    boost::array<double,200> active;
    unsigned short i,j;

    for (i=0;i<200;i++)
    {
        X_EKF[i]=0;
        active[i]=0;
    }

    double cell_start = 1;
    double cell_end = 10;
    boost::array<double,3> position = {100.0,-182.0,0.0};
    boost::array<double,3> velocity = {1.0,0.5,0}; // this is in N,E,D directions. In the previous filter this is not the case.
    boost::array<double,3> euler = {0,0,0};
    double depth_cell_size = 1;
    double blank_d = 0.1;
    double vert_grid_size = 15;
    double hori_res = 50;
    double beam_pitch = M_PI/6;
    double beam_yaw = M_PI/4;
    bool failflag = false;
    double max_states = 76;
    double num_other_states = 16;
    unsigned short new_state;
    unsigned short old_state;

    double time_sim = 1000;
    double dt = 10;
    double time = 0;

//    X_EKF[199] = 5.1;

    ADCP_measurement_model ADCP_model(cell_start,depth_cell_size,blank_d,vert_grid_size,hori_res,
                                      X_EKF,max_states,num_other_states, position, velocity, euler);

    for (time=0;time<time_sim;time+=dt)
    {
        position[0] += velocity[0] * dt;
        position[1] += velocity[1] * dt;
        position[2] += velocity[2] * dt;
        std::cout << "Position: " << position[0] << " "<< position[1]<< " "<< position[2] << std::endl;

        ADCP_model.resetOutput(); // zeros the Predicted measurement and Jacobian matrices, and the measurement counter

        for(i=0;i<4;i++)
        {
            ADCP_model.setBeam(cell_end,position,velocity,euler,beam_pitch,beam_yaw+i*M_PI/2);

            failflag = ADCP_model.calculateInterceptsandWeightings();

            std::cout << "failflag: " <<failflag << std::endl;

            while ((new_state=ADCP_model.isNewStates()))\
            {
                if (ADCP_model.checkRemoval())
                {
                    old_state = ADCP_model.removeOldest();

                    // below emulates the process of removing the old state from the filter (zeroing it)

                    active[old_state] = 0;
                    active[old_state+1] = 0;
                    active[old_state+2] = 0;

                    // below emulates the process of adding the state to the filter (initializing)

                    active[new_state] = 1;
                    active[new_state+1] = 1;
                    active[new_state+2] = 1;
                }
                else
                {
                    // we initialize new states, since we are filling the state vector for the first time

                    active[new_state] = 1;
                    active[new_state+1] = 1;
                    active[new_state+2] = 1;

                }

            }

            // now calculate the predicted measurements and jacobians
            // for all valid cells as communicated by cell_start and cell_end
            ADCP_model.calculatePredictedMeasurement();
            std::cout << "m_num:" << ADCP_model.getMeasurementNumber() << std::endl;
            for (j=0;j<ADCP_model.getMeasurementNumber()-1;j++)
            {
                std::cout << " " << ADCP_model.PredictedMeasurement[j];
            }
                             std::cout << std::endl;
            }
                             //ADCP_model.PredictedMeasurement;
                             //ADCP_model.Jacobian;
            }

                             //    ADCP_model.test();

                             return 0;
            }

