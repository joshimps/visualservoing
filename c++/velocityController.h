#include "ros/ros.h"

class VelocityController{
    public:

    protected:

    private:

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors and Destructors
    /////////////////////////////////////////////////////////////////////////////////////////

    VelocityController(ros::NodeHandle nh);
    ~VelocityController();


    ///////////////////////////////////////////////////////////////////////
    // Node, Publishers and Subscribers
    /////////////////////////////////////////////////////////////////////
    
    ros::NodeHandle nh_;
    
}