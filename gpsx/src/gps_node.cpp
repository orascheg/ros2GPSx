#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "gpsx/msg/gpsx.hpp"
#include "gpsx/srv/get_sat_list.hpp"

using namespace std::chrono_literals;

/*
$GPGGA,123313,4704.8062,N,01525.3878,E,1,09,1.1,359.7,M,43.7,M,,*4E
$GPGGA,HHMMSS.ss,AAAA.AAAA,la,OOOO.OOOO,lo,Q,NN,D.D,H.H,h,G.G,g,A.A,RRRR*CS

0	Message ID $GPGGA
1	HHMMSS.ss 	Time of position fix (UTC)
2	AAAA.AAAA 	Latitude degree and minutes (ddmm.mmmmmm)
3	la 	Direction of latitude (North or South)
4	OOOO.OOOO 	Longitude degree and minutes (dddmm.mmmmmm)
5	lo 	Direction of longitude (East or West)
6	Q 	GPS quality indicator:

    	0: no fix available
    	1: GPS fix
    	2: Differential GPS fix (DGPS)
      3: PPS fix
      4: Real Time Kinematic fix (RTK)
      5: Float RTK
    	6: Estimated (dead reckoning)
      7: Manual input mode
      8: simulation mode

7	NN 	Number of satellites in use (00 − 12)
8	D.D 	horizontale dilution of precision (meter)
9	H.H 	Antenna altitude (meter)
10	h 	Unit of antenna altitude (meter)
11	G.G 	geoidal separation the difference between the WGS-84 earth ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid
12	g 	Unit of geoidal separation (meter)
13	A.A 	Age of differential GPS data, time in seconds since last SC104 type 1 or 9 update, null field when DGPS is not used
14	RRRR 	Differential reference station ID (0000 to 1023)
15	CS 	Checksum
*/
struct messageGGA
{
  std::string UTCtime;
  double latitude;
  double longitude;
  double altitude;
  double dilution;
  double separation;
  int fix;
  int satellites;
};

/*
$GPVTG,,T,,M,0.00,N,0.00,K*CS	 
$GPVTG,000.0,T,356.8,M,000.0,N,0000.0,K,A*1B

0 	Message ID $GPVTG
1 	Track made good (degrees true)
2 	T: track made good is relative to true north
3 	Track made good (degrees magnetic)
4 	M: track made good is relative to magnetic north
5 	Speed over ground, in knots
6 	N: speed is measured in knots
7 	Speed over ground in kilometers/hour (kph)
8 	K: speed over ground is measured in kph
9 	The checksum data, always begins with *

*/
struct messageVTG
{
  double true_course;
  double ground_speed;
  double true_course_magnetic;
};

/*
$GPGSV,M,N,S,SV,02,213,,03,-3,000,,11,00,121,,14,13,172,05*67
$GPGSV,1,1,13,02,02,213,,03,-3,000,,11,00,121,,14,13,172,05*67


1 M: Total number of messages of this type in this cycle
2 N: Message number
3 S: Total number of SVs in view
4 SV: PRN number
5    = Elevation in degrees, 90 maximum
6    = Azimuth, degrees from true north, 000 to 359
7    = SNR, 00-99 dB (null when not tracking)
8-11 = Information about second SV, same as field 4-7
12-15= Information about third SV, same as field 4-7
16-19= Information about fourth SV, same as field 4-7
*/

struct satellite
{
  /* data */
  unsigned int id;
  int elevation; // -90 to 90 degrees
  unsigned int azimuth; // 0 to 360
  unsigned int SNR; // 0 to 100 in most cases
  unsigned int type; // 1: GPS, 2:Glonas, 3: Beidou, 4: Galileo
};

struct messageGSV
{
  /* data */
  unsigned short int msgCount; // 1 to 9
  unsigned short int currCount;
  unsigned short int satInView; 
  struct satellite sats[4];
};

// this must not be part of the class, leads to a compiler error if so
void listGps(const std::shared_ptr<gpsx::srv::GetSatList::Request> request,std::shared_ptr<gpsx::srv::GetSatList::Response> response);
std::vector <struct satellite> sat_monitor_;

class GPSPublisher : public rclcpp::Node
{
  public:
    GPSPublisher(): Node("gps_publisher"), initialized_(false), newdata_(false), run_(false)
    {
      // initialize message structures
      gga_.UTCtime=std::string();
      gga_.latitude=0.0;
      gga_.longitude=0.0;
      gga_.altitude=0.0;
      gga_.dilution=0.0;
      gga_.separation=0.0;
      gga_.fix=0;
      gga_.satellites=0;
      vtg_.true_course=0.0;
      vtg_.ground_speed=0.0;
      vtg_.true_course_magnetic=0.0;

      // declare parameters
      this->declare_parameter<std::string>("comm_port", "/dev/ttyS0");
      rcl_interfaces::msg::IntegerRange range;
      range.from_value = 4800;
      range.step = 1;
      range.to_value = 115200;
      rcl_interfaces::msg::ParameterDescriptor comm_speed_descriptor;
      comm_speed_descriptor.description = "Serial interface speed setting in Baud";
      comm_speed_descriptor.integer_range.push_back(range);
      this->declare_parameter("comm_speed", rclcpp::ParameterValue(4800), comm_speed_descriptor);
      run();
    }
    ~GPSPublisher()
    {
      gpsConnection_.close();
    }
    
  private:
    void timer_callback();
    void run();
    double safe_stod(std::string& convert);
    double convert_longlat(std::string& convert);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<gpsx::msg::Gpsx>::SharedPtr publisher_;
    rclcpp::Service<gpsx::srv::GetSatList>::SharedPtr service_;
    int openConnection(void);
    int closeConnection(void);
    int readMessage(void);
    int preprocessMessage(std::string* message);
    
    std::fstream gpsConnection_;
    struct messageGGA gga_;
    struct messageVTG vtg_;
    struct messageGSV gsv_;
    
    bool initialized_;
    bool newdata_;
    bool run_;

};
 
void GPSPublisher::run(void)
{
  publisher_ = this->create_publisher<gpsx::msg::Gpsx>("gpsx", 10);
  service_ = this->create_service<gpsx::srv::GetSatList>("get_sat_list",&listGps);
  timer_ = this->create_wall_timer(50ms, std::bind(&GPSPublisher::timer_callback, this));
}
 

// returns 0 on success and negative number on error
int GPSPublisher::openConnection(void)
{
  struct termios tty;
  int fd;
  std::string serial_portp;
  int serial_speed;
  
  this->get_parameter("comm_port", serial_portp);
  this->get_parameter("comm_speed", serial_speed);
  RCLCPP_INFO(this->get_logger(), "Opening port: %s speed: %d",serial_portp.c_str(),serial_speed);

  fd = open (serial_portp.c_str(), O_RDWR | O_NOCTTY);
  if(fd<0)
    return -2;
    
  // get the current setting, we assume that 8N1 is set TODO: fix so that this is always set
  if(tcgetattr(fd, &tty) != 0)
    RCLCPP_INFO(this->get_logger(), "Error %d from tcgetattr: %s", errno, strerror(errno));
  tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;         /* 8-bit characters */
  tty.c_cflag &= ~PARENB;     /* no parity bit */
  tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
  tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

  // setup for non-canonical mode
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  // fetch bytes as they become available
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;
  
  // set baud rate to the parameter comm_speed
  if(serial_speed==4800)
    cfsetispeed(&tty, B4800);
  else if(serial_speed==9600)
    cfsetispeed(&tty, B9600);    
  else if(serial_speed==19200)
    cfsetispeed(&tty, B19200);
  else if(serial_speed==38400)
    cfsetispeed(&tty, B38400);
  else if(serial_speed==57600)
    cfsetispeed(&tty, B57600);
  else if(serial_speed==115200)
    cfsetispeed(&tty, B115200);

  
  if (tcsetattr(fd, TCSANOW, &tty) != 0)
    RCLCPP_ERROR(this->get_logger(), "Error %d from tcsetattr: %s",errno,strerror(errno));
  close(fd);

  gpsConnection_.open(serial_portp.c_str(), std::fstream::in | std::fstream::out);
  if(!gpsConnection_.is_open())
  {
    RCLCPP_ERROR(this->get_logger(),"Input stream could not be opened!");
    return -1;    
  }
  
  initialized_=true;
  RCLCPP_INFO(this->get_logger(),"Successfully opened serial connection to GPS");

  return 0;
}

// returns 0 on success <0 on error
int GPSPublisher::closeConnection(void)
{
  if(gpsConnection_.is_open())
  {
    gpsConnection_.close();
  }
  initialized_=false;
  return 0;
}

// Returns the number in the string or NAN on error
double GPSPublisher::safe_stod(std::string& convert)
{
  try
  {          
    return(std::stod(convert));
  }
  catch(const std::invalid_argument& ia)
  {
    //RCLCPP_WARN(this->get_logger(),"Could not convert" + convert + " reason: "+ std::string(ia.what()));
    return(NAN); 
  }      
}

// returns calculated longitude or latitude, NAN on conversion error
double GPSPublisher::convert_longlat(std::string& convert)
{
  double value=safe_stod(convert);
  if(value==NAN)
    return value;
  double dDegrees=floor(value/100);
  double dMinutes=fmod(value,100);
  return(dDegrees+dMinutes/60.0);
}

// returns 0 on success
// -1: invalid start character
// -2: No asterisk indicating start of checksum
// -3: Checksum does not match
int GPSPublisher::preprocessMessage(std::string* message)
{
  // check checksum 8 Bit XOR for everything between (not including) $ and *
  short int cs_calculated=0;
  short int cs_recieved=0;
  size_t found=0;
  int count=0;

  if((message->at(0)!='$')&&(message->at(0)!='!'))
    return -1;
  found=message->find('*');
  if(found==0)
    return -2;
  cs_recieved=strtol((message->substr(found+1,2)).c_str(),NULL,16);
  for(count=1;count<(int)found;count++)
    cs_calculated ^= message->at(count);
  if(cs_calculated!=cs_recieved)
    return -3;
  // replace the ',,' for empty values with ', ,' for following tokenization
  do
  {
    found=message->find(",,");
    if(found!=std::string::npos)
      message->replace(found,2,", ,");
  }
  while(found!=std::string::npos);

  return 0;
}

// returns 0 on success <0 on error
int GPSPublisher::readMessage(void)
{
  if(initialized_)
  {
    std::string msgRead;
    std::vector <std::string> tokens;
    std::string intermediate;
    
    int c = gpsConnection_.peek();  // peek character
    if(c==EOF)
      return -10;
  
    std::getline(gpsConnection_,msgRead);
    if(preprocessMessage(&msgRead)<0)
    {
      std::cout << "Check of message read failed: " << msgRead << std::endl;
      return -11;
    }

    // check if proper token
    // $GP: GPS reciever
    // $GN: Combination of different satellite positioning systems
    // $GL: Glonass reciever
    // $BD: Beidou reciever
    // $GA: Galileo reciever
    if(msgRead.compare(0,3,"$GP")!=0&&msgRead.compare(0,3,"$GN")!=0&&msgRead.compare(0,3,"$GL")&&msgRead.compare(0,3,"$BD")&&msgRead.compare(0,3,"$GA"))
    {
      RCLCPP_ERROR(this->get_logger(),"unknown start token of message: %s",msgRead.c_str());
    }
    //RCLCPP_INFO(this->get_logger(), "Read line: "+ msgRead);    
    
    // create stringstream for tokenization
    std::stringstream check1(msgRead);
    // Tokenizing with ',' 
    while(getline(check1, intermediate, ',')) 
      tokens.push_back(intermediate); 

    // is this one of the implemented messages?
    if(msgRead.compare(3,3,std::string("GGA"))==0)
    {
      //std::cout << "Got new GGA data" << std::endl;
      // this is the GGA message
      for(unsigned int i=1;i<tokens.size();i++)
      {
        switch(i)
        {
          case 0: // will not happen...
        
          break;
          case 1: // this is UTC time
            gga_.UTCtime= tokens[i];
          break;
          case 2: // this is latitude
            gga_.latitude=convert_longlat(tokens[i]);
          break;
          case 3: // orientation North or South
            if((tokens[i].compare("N")!=0) and (tokens[i].compare("n")!=0))
              // southern hemisphere
              gga_.latitude*=-1;
          break;
          case 4: // this is longitude
            gga_.longitude=convert_longlat(tokens[i]);
          break;
          case 5: // orientation East or West
            if((tokens[i].compare("E")!=0) and (tokens[i].compare("e")!=0))
              // western hemisphere
              gga_.longitude*=-1;
          break;
          case 6: // quality of fix, 0 no fix, 1 fix, 2 ground augmented
            gga_.fix=safe_stod(tokens[i]);
          break;
          case 7: // number of sattelites
            gga_.satellites=safe_stod(tokens[i]);
          break;
          case 8: // horizontal dilution of precision
            gga_.dilution=safe_stod(tokens[i]);
          break;
          case 9: // altitude
            gga_.altitude=safe_stod(tokens[i]);
          break;
          case 10: // unit of altitude, usually Meter
            if((tokens[i].compare("M")!=0) and (tokens[i].compare("m")!=0))
              RCLCPP_ERROR(this->get_logger(),"GPS does not report altitude in meter! %s",tokens[i].c_str()); 
          break;
          case 11: // geoidal separation
            gga_.separation=safe_stod(tokens[i]);
          break;
          case 12:
            if((tokens[i].compare("M")!=0) and (tokens[i].compare("m")!=0))
              RCLCPP_ERROR(this->get_logger(),"GPS does not report geoidal separation in meter! %s",tokens[i].c_str());           
          break;
          // ignoring the rest
        }
      }
      //RCLCPP_INFO(this->get_logger(),"UTCtime: "+ UTCtime +" fix:" + std::to_string(fix) + " satellites: " + std::to_string(satellites) + " dilution: " + std::to_string(dilution) + " separation: "+ std::to_string(separation));
      std::cout << "UTCtime: " << gga_.UTCtime << " fix:" << std::to_string(gga_.fix) << " satellites: " << std::to_string(gga_.satellites) << " dilution: " << std::to_string(gga_.dilution) << " separation: " << std::to_string(gga_.separation) << " latitude: " << std::to_string(gga_.latitude) << " longitude: " << std::to_string(gga_.longitude) << std::endl;
    }
    // else if(strncmp(readBuffer,"$GPVTG",6)==0)
    else if(msgRead.compare(3,3,std::string("VTG"))==0)
    {
      // this is the VTG message
      newdata_=true;
      for(unsigned int i=1;i<tokens.size();i++)
      {
        switch(i)
        {
          case 0: // will not happen
            // ignore
          break;
          case 1: // Track made good in degrees geographical
            vtg_.true_course=safe_stod(tokens[i]);
          break;
          case 2: // direction of true course
            if((tokens[i].compare("T")!=0) and (tokens[i].compare("t")!=0))
              RCLCPP_ERROR(this->get_logger(),"GPS does not report true course to north! %s",tokens[i].c_str()); 
          break;
          case 3: // Track made good in degrees magnetic
            vtg_.true_course_magnetic=safe_stod(tokens[i]);
          break;
          case 4: // direction of true course
            if((tokens[i].compare("M")!=0) and (tokens[i].compare("m")!=0))
              RCLCPP_ERROR(this->get_logger(),"GPS does not report true course to magnetic north! %s",tokens[i].c_str()); 
          break;
          case 5: // speed in knots
            // ignore
          break;
          case 6: // speed unit
            // ignore
          break;
          case 7: // ground speed in km/h
            vtg_.ground_speed=safe_stod(tokens[i]);
          break;
          case 8: // direction of true course
            if((tokens[i].compare("K")!=0) and (tokens[i].compare("k")!=0))
              RCLCPP_ERROR(this->get_logger(),"GPS does not report ground speed in km/h! %s",tokens[i].c_str()); 
          break;                              
        }
      }
      std::cout << "true course: " << std::to_string(vtg_.true_course) << " magnetic:" << std::to_string(vtg_.true_course_magnetic) << " ground speed: " << std::to_string(vtg_.ground_speed) << std::endl;
    }
    // Satellites in view message, due to many satellites this is provided with multiple satellites (4 per line)
    else if(msgRead.compare(3,3,std::string("GSV"))==0)
    {
      //std::cout << "message read test test: " << msgRead << std::endl;
      if(msgRead.compare(1,2,"GP")==0)
      {
        // initialize storage to 0
        memset(&gsv_,0,sizeof(struct messageGSV));
        // these are GPS satellites
        for(unsigned int i=1;i<tokens.size();i++)
        {
          switch(i)
          {
            case 0: // will not happen
              // ignore
            break;
            case 1: // Total number of sentences in group
              gsv_.msgCount=safe_stod(tokens[i]);
            break;
            case 2:
              gsv_.currCount=safe_stod(tokens[i]);
              // re-initialize the overall storage of satellites, always done on the first message of GPS sats
              if(1==gsv_.currCount)
                sat_monitor_.clear();
            break;
            case 3:
              gsv_.satInView=safe_stod(tokens[i]);
            break;
            case 4:
              gsv_.sats[0].id=safe_stod(tokens[i]);
            break;
            case 5:
              gsv_.sats[0].azimuth=safe_stod(tokens[i]);
            break;
            case 6:
              gsv_.sats[0].elevation=safe_stod(tokens[i]);
            break;
            case 7:
              gsv_.sats[0].SNR=safe_stod(tokens[i]);
              gsv_.sats[0].type=1;             
            break;  
            case 8:
              gsv_.sats[1].id=safe_stod(tokens[i]);
            break;
            case 9:
              gsv_.sats[1].azimuth=safe_stod(tokens[i]);
            break;
            case 10:
              gsv_.sats[1].elevation=safe_stod(tokens[i]);
            break;
            case 11:
              gsv_.sats[1].SNR=safe_stod(tokens[i]);
              gsv_.sats[1].type=1;       
            break;  
            case 12:
              gsv_.sats[2].id=safe_stod(tokens[i]);
            break;
            case 13:
              gsv_.sats[2].azimuth=safe_stod(tokens[i]);            
            break;
            case 14:
              gsv_.sats[2].elevation=safe_stod(tokens[i]);            
            break;
            case 15:
              gsv_.sats[2].SNR=safe_stod(tokens[i]);            
              gsv_.sats[2].type=1;             
            break;  
            case 16:
              gsv_.sats[3].id=safe_stod(tokens[i]);
            break;
            case 17:
              gsv_.sats[3].azimuth=safe_stod(tokens[i]);            
            break;
            case 18:
              gsv_.sats[3].elevation=safe_stod(tokens[i]);            
            break;
            case 19:
              gsv_.sats[3].SNR=safe_stod(tokens[i]);            
              gsv_.sats[3].type=1;             
            break;  
          }
        }
        std::cout << "Number of GPS satellites: "<< gsv_.satInView << " Message number: " << gsv_.currCount << " ID sat 1 " << gsv_.sats[0].id << " ID sat 2 " << gsv_.sats[1].id << " ID sat 3 " << gsv_.sats[2].id << " ID sat 4 " << gsv_.sats[3].id << std::endl;
        int limit=0;
        // decide how many satellites are to be copied
        if(gsv_.currCount<gsv_.msgCount)
          limit=4;
        else
          limit=4-((gsv_.currCount)*4-gsv_.satInView);
        for(int count=0;count<limit;count++)
          sat_monitor_.push_back(gsv_.sats[count]);
      }
      else if(msgRead.compare(1,2,"GL")==0)
      {
        // these are the Glonass satellites
        memset(&gsv_,0,sizeof(struct messageGSV));
        // these are GPS satellites
        for(unsigned int i=1;i<tokens.size();i++)
        {
          switch(i)
          {
            case 0: // will not happen
              // ignore
            break;
            case 1: // Total number of sentences in group
              gsv_.msgCount=safe_stod(tokens[i]);
            break;
            case 2:
              gsv_.currCount=safe_stod(tokens[i]);
            break;
            case 3:
              gsv_.satInView=safe_stod(tokens[i]);
            break;
            case 4:
              gsv_.sats[0].id=safe_stod(tokens[i]);
            break;
            case 5:
              gsv_.sats[0].azimuth=safe_stod(tokens[i]);
            break;
            case 6:
              gsv_.sats[0].elevation=safe_stod(tokens[i]);
            break;
            case 7:
              gsv_.sats[0].SNR=safe_stod(tokens[i]);
              gsv_.sats[0].type=2;             
            break;  
            case 8:
              gsv_.sats[1].id=safe_stod(tokens[i]);
            break;
            case 9:
              gsv_.sats[1].azimuth=safe_stod(tokens[i]);
            break;
            case 10:
              gsv_.sats[1].elevation=safe_stod(tokens[i]);
            break;
            case 11:
              gsv_.sats[1].SNR=safe_stod(tokens[i]);
              gsv_.sats[1].type=2;       
            break;  
            case 12:
              gsv_.sats[2].id=safe_stod(tokens[i]);
            break;
            case 13:
              gsv_.sats[2].azimuth=safe_stod(tokens[i]);            
            break;
            case 14:
              gsv_.sats[2].elevation=safe_stod(tokens[i]);            
            break;
            case 15:
              gsv_.sats[2].SNR=safe_stod(tokens[i]);            
              gsv_.sats[2].type=2;             
            break;  
            case 16:
              gsv_.sats[3].id=safe_stod(tokens[i]);
            break;
            case 17:
              gsv_.sats[3].azimuth=safe_stod(tokens[i]);            
            break;
            case 18:
              gsv_.sats[3].elevation=safe_stod(tokens[i]);            
            break;
            case 19:
              gsv_.sats[3].SNR=safe_stod(tokens[i]);            
              gsv_.sats[3].type=2;             
            break;  
          }
        }
        int limit=0;
        // decide how many satellites are to be copied
        if(gsv_.currCount<gsv_.msgCount)
          limit=4;
        else
          limit=4-((gsv_.currCount)*4-gsv_.satInView);
        for(int count=0;count<limit;count++)
          sat_monitor_.push_back(gsv_.sats[count]);
      }
      else if(msgRead.compare(1,2,"GA")==0)
      {
        // these are the Galileo satellites
        std::cout << "message read GA: " << msgRead << std::endl;
      }
    }
    else
    {
      std::cout << "Unknown message: " << msgRead << std::endl;
    }
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Parameterized interface not initialized");
    return -1;
  }
  return 0;
}

void GPSPublisher::timer_callback()
{
  auto message = gpsx::msg::Gpsx();
  if(!initialized_)
  {
    // retry to establish connection in 1 second
    RCLCPP_INFO(this->get_logger(), "Retry to open port on next timer slot second"); 
    if(openConnection()<0)
      return;
  }
  // read and interprete the messages recieved
  readMessage();


  if(newdata_)
  {
    message.longitude=gga_.longitude;
    message.latitude=gga_.latitude;
    message.altitude=gga_.altitude;
    message.satellites=gga_.satellites;
    message.mode_indicator=gga_.fix;
    message.separation=gga_.separation;
    message.dilution=gga_.dilution;
    if(gga_.UTCtime.length()>0)
    {
      //std::cout << "Converting UTC: "<< UTCtime << std::endl;
      message.utc_time=std::strtod(gga_.UTCtime.c_str(),0);
    }
    message.true_course_magnetic=vtg_.true_course_magnetic;
    message.true_course=vtg_.true_course;
    message.ground_speed=vtg_.ground_speed;
    //RCLCPP_INFO(this->get_logger(), "Publishing: long: '%f' lat: '%f' alt: '%f'", message.longitude, message.latitude, message.altitude);
    publisher_->publish(message);
    newdata_=false;
  }
}

void listGps(const std::shared_ptr<gpsx::srv::GetSatList::Request> request,std::shared_ptr<gpsx::srv::GetSatList::Response> response)
{
  std::cout << "Got request: " << request->type;
  std::vector<int16_t> ids;
  std::vector<int16_t> types;
  std::vector<int16_t> elevations;
  std::vector<int16_t> azimuths;
  std::vector<int16_t> snrs;    

  for(auto it : sat_monitor_)
  {
    ids.push_back(it.id);
    types.push_back(it.type);
    elevations.push_back(it.elevation);
    azimuths.push_back(it.azimuth);
    snrs.push_back(it.SNR);
  }
  response->id=std::move(ids);
  response->gnsstype=std::move(types);
  response->elevation=std::move(elevations);
  response->azimuth=std::move(azimuths);
  response->snr=std::move(snrs);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSPublisher>());
  rclcpp::shutdown();
  return 0;
}
