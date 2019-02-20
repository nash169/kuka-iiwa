#include <kuka_iiwa/iiwa_hardware_interface.h>

#define DEFAULT_PORTID 30200

int main(int argc, char** argv)
{

    /***************************************************************************/
    /*                                                                         */
    /*   Place user Client Code here                                           */
    /*                                                                         */
    /***************************************************************************/

    ros::init(argc, argv, "iiwa_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // create new client
    iiwa_hardware_interface::IIWAHardwareInterface iiwa(nh);

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure                                        */
    /*   Configuration                                                         */
    /*                                                                         */
    /***************************************************************************/

    // create new udp connection
    kuka::fri::UdpConnection connection;

    // pass connection and client to a new FRI client application
    kuka::fri::ClientApplication app(connection, iiwa);

    // Connect client application to KUKA Sunrise controller.
    // Parameter NULL means: repeat to the address, which sends the data
    app.connect(DEFAULT_PORTID, "192.170.10.2");

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure                                        */
    /*   Execution mainloop                                                    */
    /*                                                                         */
    /***************************************************************************/

    // repeatedly call the step routine to receive and process FRI packets
    bool success = true;
    while (success && ros::ok()) {
        success = app.step();
    }

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure                                        */
    /*   Dispose                                                               */
    /*                                                                         */
    /***************************************************************************/

    // disconnect from controller
    app.disconnect();

    spinner.stop();

    return 0;
}