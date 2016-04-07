#include <boost/bind.hpp>

// ROS includes
#include <px_comm/OpticalFlow.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "px4flow/SerialComm.h"

namespace px
{

SerialComm::SerialComm(const std::string& frameId)
 : m_port(m_uartService)
 , m_timer(m_uartService)
 , m_imageSize(0)
 , m_imagePackets(0)
 , m_imagePayload(0)
 , m_imageWidth(0)
 , m_imageHeight(0)
 , m_systemId(-1)
 , m_compId(0)
 , m_frameId(frameId)
 , m_timeout(false)
 , m_errorCount(0)
 , m_connected(false)
 , ros_time_init(ros::Time::now())
 , stamp(ros_time_init)
 , time_diff(0)
 , time_ok(false)
 , nstep(0)
{
}

SerialComm::~SerialComm()
{
    if (m_connected)
    {
        close();
    }
}

bool
SerialComm::open(const std::string& portStr, int baudrate)
{
    m_timeout = false;
    m_errorCount = 0;
    m_connected = false;

    // open port
    try
    {
        m_port.open(portStr);

        ROS_INFO("Opened serial port %s.", portStr.c_str());
    }
    catch (boost::system::system_error::exception e)
    {
        ROS_ERROR("Could not open serial port %s. Reason: %s.", portStr.c_str(), e.what());

        return false;
    }

    // configure baud rate
    try
    {
        m_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        m_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        m_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        m_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        m_port.set_option(boost::asio::serial_port_base::character_size(8));

        ROS_INFO("Set baudrate %d.", baudrate);
    }
    catch (boost::system::system_error::exception e)
    {
        ROS_ERROR("Could not set baudrate %d. Reason: %s.", baudrate, e.what());

        return false;
    }

    ros::NodeHandle nh("px4flow");

    // set up publishers
    m_optFlowPub = nh.advertise<px_comm::OpticalFlow>("opt_flow", 5);

    image_transport::ImageTransport it(nh);
    m_imagePub = it.advertise("camera_image", 5);

    // set up thread to asynchronously read data from serial port
    readStart(1000);
    m_uartThread = boost::thread(boost::bind(&boost::asio::io_service::run, &m_uartService));

    m_syncTimer = nh.createTimer(ros::Duration(2.0), boost::bind(&SerialComm::syncCallback, this, _1));

    m_connected = true;

    return true;
}

void
SerialComm::close(void)
{
    if (!m_connected)
    {
        return;
    }

    m_uartService.post(boost::bind(&boost::asio::deadline_timer::cancel, &m_timer));
    m_uartService.post(boost::bind(&boost::asio::serial_port::close, &m_port));

    m_uartThread.join();
}

void
SerialComm::readCallback(const boost::system::error_code& error, size_t bytesTransferred)
{
    if (error)
    {
        if (error == boost::asio::error::operation_aborted)
        {
            // if serial connection timed out, try reading again
            if (m_timeout)
            {
                m_timeout = false;
                readStart(1000);

                return;
            }
        }

        ROS_WARN("Read error: %s", error.message().c_str());

        if (m_errorCount < 10)
        {
            readStart(1000);
        }
        else
        {
            ROS_ERROR("# read errors exceeded 10. Aborting...");
            ros::shutdown();
        }

        ++m_errorCount;

        return;
    }

    m_timer.cancel();

    mavlink_message_t message;
    mavlink_status_t status;

    for (size_t i = 0; i < bytesTransferred; i++) {
        bool msgReceived = mavlink_parse_char(MAVLINK_COMM_1, m_buffer[i], &message, &status);

        if (msgReceived)
        {
            m_systemId = message.sysid;

            switch (message.msgid)
            {
            case MAVLINK_MSG_ID_OPTICAL_FLOW:
            {
                // decode message
                mavlink_msg_optical_flow_decode(&message, &flow);


                // The sensor time has a weird behaviour and we have to store 
                // the initial sensor time on third loop step.
                if (!time_ok)
                {
                    if (nstep >= 3)
                    {
                        sensor_ini_usec = flow.time_usec;   
                        ros_time_init = ros::Time::now();  
                        time_ok = true;
                    }
                    else
                        nstep = nstep + 1;
                }
                else
                {
                  int sensor_time_usec = flow.time_usec - sensor_ini_usec;
                  stamp = ros_time_init + ros::Duration(sensor_time_usec / 1000000, (sensor_time_usec % 1000000) * 1000);
        
                  ros::Duration ros_time = ros::Time::now()-ros_time_init;
                  ros::Duration sensor_time = stamp-ros_time_init;
        
                  if (ros_time.toSec() < 1.0)
                      time_diff = sensor_time - ros_time;
        
                  // // DEBUG
                  // std::cout << "**************" << std::endl;
                  // std::cout << flow.time_usec << std::endl;
                  // std::cout << sensor_time_usec << std::endl;
                  // std::cout << ros_time.toSec() << "  ROS ---" << std::endl;
                  // std::cout << sensor_time.toSec() << "  Sensor ***" << std::endl;
                  // std::cout << time_diff.toSec() << " Diff ...." << std::endl;
                  // std::cout << (stamp - time_diff - ros_time_init).toSec() << " Stamp ...." << std::endl;          

                  // Publish message
                  px_comm::OpticalFlow optFlowMsg;                
                  optFlowMsg.header.stamp = stamp - time_diff;
                  // optFlowMsg.header.stamp = ros::Time(flow.time_usec / 1000000, (flow.time_usec % 1000000) * 1000);
                  optFlowMsg.header.frame_id = m_frameId;
                  optFlowMsg.ground_distance = flow.ground_distance;
                  optFlowMsg.flow_x = flow.flow_x;
                  optFlowMsg.flow_y = flow.flow_y;
                  optFlowMsg.velocity_x = flow.flow_comp_m_x;
                  optFlowMsg.velocity_y = flow.flow_comp_m_y;
                  optFlowMsg.quality = flow.quality;

                  m_optFlowPub.publish(optFlowMsg);
                }

                break;
            }
            case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
            {
                mavlink_data_transmission_handshake_t handshake;
                mavlink_msg_data_transmission_handshake_decode(&message, &handshake);

                m_imageSize = handshake.size;
                m_imagePackets = handshake.packets;
                m_imagePayload = handshake.payload;
                m_imageWidth = handshake.width;
                m_imageHeight = handshake.height;

                if (m_imageBuffer.size() < m_imageSize)
                {
                    m_imageBuffer.resize(m_imageSize);
                }

                break;
            }
            case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
            {
                if (m_imageSize == 0 || m_imagePackets == 0)
                {
                    break;
                }

                mavlink_encapsulated_data_t img;
                mavlink_msg_encapsulated_data_decode(&message, &img);
                size_t seq = img.seqnr;
                size_t pos = seq * m_imagePayload;

                if (seq + 1 > m_imagePackets)
                {
                    break;
                }

                size_t bytesToCopy = m_imagePayload;
                if (pos + m_imagePayload >= m_imageSize)
                {
                     bytesToCopy = m_imageSize - pos;
                }

                memcpy(&m_imageBuffer[pos], img.data, bytesToCopy);

                if (seq + 1 == m_imagePackets)
                {
                  if (time_ok)
                  {
                    sensor_msgs::Image image;
                    image.header.stamp = stamp - time_diff;
                    image.header.frame_id = m_frameId;
                    image.height = m_imageHeight;
                    image.width = m_imageWidth;
                    image.encoding = sensor_msgs::image_encodings::MONO8;
                    image.is_bigendian = false;
                    image.step = m_imageWidth;

                    image.data.resize(m_imageSize);
                    memcpy(&image.data[0], &m_imageBuffer[0], m_imageSize);

                    m_imagePub.publish(image);
                  }
                }
                break;
            }
            }
        }
    }

    readStart(1000);
}

void
SerialComm::readStart(uint32_t timeout_ms)
{
    m_port.async_read_some(boost::asio::buffer(m_buffer, sizeof(m_buffer)),
                            boost::bind(&SerialComm::readCallback, this, boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));

    if (timeout_ms != 0)
    {
        m_timer.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
        m_timer.async_wait(boost::bind(&SerialComm::timeoutCallback, this, boost::asio::placeholders::error));
    }
}

void
SerialComm::syncCallback(const ros::TimerEvent& timerEvent)
{
    if (m_systemId == -1)
    {
        return;
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);

    uint64_t timeNow = static_cast<uint64_t>(tv.tv_sec) * 1000000 + tv.tv_usec;

    mavlink_message_t msg;
    mavlink_msg_system_time_pack(m_systemId, m_compId, &msg, timeNow, 0);

    size_t messageLength = mavlink_msg_to_send_buffer(m_buffer, &msg);
    if (messageLength != boost::asio::write(m_port, boost::asio::buffer(m_buffer, messageLength)))
    {
        ROS_WARN("Unable to send system time over serial port.");
    }
}

void
SerialComm::timeoutCallback(const boost::system::error_code& error)
{
    if (!error)
    {
        m_port.cancel();
        m_timeout = true;
        ROS_WARN("Serial connection timed out.");
    }
}

}
