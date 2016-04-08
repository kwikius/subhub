
#include <gps/gps.hpp>

quan_apm::gps::gps()
{

}

struct my_gps : quan_apm::gps {
     my_gps(): quan_apm::gps(){}
     position_type         get_position()const { return position_type{};}
     position_error_type   get_position_error() const  { return position_error_type{};}
     velocity3d_type       get_velocity3d()const{ return velocity3d_type{};}
     int                   get_num_sats()const { return -1;}
     gps_time              get_gps_time()const { return gps_time{};}

};

const quan_apm::gps* quan_apm::get_gps()
{
   return new  my_gps;
}

