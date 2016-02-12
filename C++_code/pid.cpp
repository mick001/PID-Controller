// PID regulator

#include<iostream>
#include<fstream>

using namespace std;

//PID class
class PID
{
private:

    double kp;
    double ki;
    double kd;

    double set_point = 0;
    bool antiwindup = false;
    double windupMax;

    double iTerm=0;

    double sample_time = 0.02;
    double dt = 0.02;
    double now_time=0;
    double old_time=0;

    double outValue=0;
    double last_error = 0;
    double last_y = 0;

public:

    // Constructor
    PID(double kp_,double ki_,double kd_,double windupMax_=0)
    {
        kp = kp_;
        ki = ki_;
        kd = kd_;

        windupMax = windupMax_;
        if(windupMax_ != 0)
        {
            antiwindup = true;
        }else
        {
            antiwindup = false;
        }
    }

    // PID output according to y_measured
    double output(double y_measured)
    {
        double error = set_point - y_measured;

        //Get elapsed time

        iTerm = iTerm + ki * error * dt;
        iTerm = antiWindup(iTerm);
        double out = kp * error + iTerm + kd * (last_y - y_measured)/dt;

        last_error = error;
        last_y = y_measured;
        return out;
    }

    // Set the setpoint
    void set_set_point(double set_point_)
    {
        set_point = set_point_;
    }

    // Get the setpoint (print it)
    void get_set_point()
    {
        cout << set_point << endl;
    }

    // Print the parameters
    void printParameters()
    {
        cout << "kp" << endl;
        cout << kp << endl;
        cout << "ki" << endl;
        cout << ki << endl;
        cout << "kd" << endl;
        cout << kd << endl;
        cout << "Windup limit" << endl;
        cout << windupMax << endl;
        cout << "Bool has windup" << endl;
        cout << antiwindup << endl;
        cout << "Set point" << endl;
        cout << set_point << endl;
    }

    // Anti windup system
    double antiWindup(double u)
    {
        if(!antiwindup)
        {
            return u;
        }else if (u >= windupMax)
        {
            return windupMax;
        }else if (u <= -windupMax)
        {
            return -windupMax;
        }else
        {
            return u;
        }

    }
};

// Save an array of data to a .csv file
void saveArray(double arrayToSave[])
{
    ofstream arrayData("C:\\users\\file1.csv");
    for(int k=0;k < 200; k++)
    {
        arrayData << arrayToSave[k] << "," << endl;
    }
    cout << "File saved" << endl;
}

// Main
int main()
{
    // PID regulator
    PID reg (1.1,1,0.001,100);

    reg.printParameters();

    cout << " "<<endl;
    cout << "Anti windup test" << endl;
    cout << reg.antiWindup(20) << endl;
    cout << reg.antiWindup(-20) << endl;
    cout << reg.antiWindup(2) << endl;

    cout << "Set point test" << endl;
    reg.get_set_point();
    reg.set_set_point(10);
    reg.get_set_point();
    cout << " " << endl;
    reg.set_set_point(0);

    cout << "PID test loop starts now..." << endl;
    cout << "Setting set point to 5..." << endl;

    double y = 0;
    double y_[200];

    // PID regulator for test
    PID reg_ (1.1,1,0.001);
    // PID test loop
    for(int i=0; i < 200; i++)
    {
        y= y + reg_.output(y);
        if (i == 50)
        {
            reg_.set_set_point(5);
        }
        if(i == 100)
        {
            reg_.set_set_point(0);
        }
        y_[i] = y;
    }

    // Save array to external file
    saveArray(y_);

    return 0;
}
