#include "gps_proto.h"
#include <string.h>
#include <functional>

void gps_proto_serialize_from_match(gps_protocol_and_message& match, gps::GpsPack* proto, gps_parsed_data_t* data){
    switch (match.protocol)
    {
        case GPS_PROTOCOL_TYPE_NMEA:
            switch (match.message)
            {
                case GPS_NMEA_TYPE_GGA:
                    gps_serialize_gga(proto->add_gga(), &data->gga);
                    break;
                case GPS_NMEA_TYPE_VTG:
                    gps_serialize_vtg(proto->add_vtg(), &data->vtg);
                    break;
                case GPS_NMEA_TYPE_GSA:
                    gps_serialize_gsa(proto->add_gsa(), &data->gsa);
                    break;
                default:
                    break;
            }
        break;
        case GPS_PROTOCOL_TYPE_UBX:
            switch (match.message)
            {
                case GPS_UBX_TYPE_NAV_DOP:
                    gps_serialize_dop(proto->add_dop(), &data->dop);
                break;
                case GPS_UBX_TYPE_NAV_PVT:
                    gps_serialize_pvt(proto->add_pvt(), &data->pvt);
                break;
                case GPS_UBX_TYPE_NAV_HPPOSECEF:
                    gps_serialize_hpposecef(proto->add_hpposecef(), &data->hpposecef);
                break;
                case GPS_UBX_TYPE_NAV_HPPOSLLH:
                    gps_serialize_hpposllh(proto->add_hpposllh(), &data->hpposllh);
                break;
                default:
                    break;
            }
        break;
        default:
        break;
    }
}
void gps_proto_deserialize(gps::GpsPack* proto, gps_proto_pack* pack, uint64_t resample_us){
    for(int i = 0; i < proto->gga_size(); i++){
        static gps_nmea_gga_t instance;
        static uint64_t last_timestamp = 0;

        instance._timestamp = proto->gga(i)._inner_timestamp();
        if(instance._timestamp - last_timestamp < resample_us)
            continue;
        else
            last_timestamp = instance._timestamp;
        if(proto->gga(i).time().size() > 0)
            memcpy(instance.time, proto->gga(i).time().c_str(), 9);
        instance.latitude = proto->gga(i).latitude();
        instance.north_south = proto->gga(i).north_south()[0];
        instance.longitude = proto->gga(i).longitude();
        instance.east_ovest = proto->gga(i).east_ovest()[0];
        instance.fix = proto->gga(i).fix();
        instance.satellites = proto->gga(i).satellites();
        instance.horizontal_diluition_precision = proto->gga(i).horizontal_diluition_precision();
        instance.fix_state = FIX_STATE[proto->gga(i).fix()];
        instance.altitude = proto->gga(i).altitude();
        instance.age_of_correction = proto->gga(i).age_of_correction();
        pack->gga.push(instance);
    }
    for(int i = 0; i < proto->vtg_size(); i++){
        static gps_nmea_vtg_t instance;
        static uint64_t last_timestamp = 0;

        instance._timestamp = proto->vtg(i)._inner_timestamp();
        if(instance._timestamp - last_timestamp < resample_us)
            continue;
        else
            last_timestamp = instance._timestamp;
        
        instance.course_over_ground_degrees = proto->vtg(i).course_over_ground_degrees();
        instance.course_over_ground_degrees_magnetic = proto->vtg(i).course_over_ground_degrees_magnetic();
        instance.speed_kmh = proto->vtg(i).speed_kmh();
        pack->vtg.push(instance);
    }
    for(int i = 0; i < proto->gsa_size(); i++){
        static gps_nmea_gsa_t instance;
        static uint64_t last_timestamp = 0;

        instance._timestamp= proto->gsa(i)._inner_timestamp();
        if(instance._timestamp - last_timestamp < resample_us)
            continue;
        else
            last_timestamp = instance._timestamp;
        
        instance.mode = proto->gsa(i).mode()[0];
        instance.position_diluition_precision = proto->gsa(i).position_diluition_precision();
        instance.horizontal_diluition_precision = proto->gsa(i).horizontal_diluition_precision();
        instance.vertical_diluition_precision = proto->gsa(i).vertical_diluition_precision();
        pack->gsa.push(instance);
    }

    for(int i = 0; i < proto->dop_size(); i++){
        static gps_ubx_dop_t instance;
        static uint64_t last_timestamp = 0;

        instance._timestamp = proto->dop(i)._inner_timestamp();
        if(instance._timestamp - last_timestamp < resample_us)
            continue;
        else
            last_timestamp = instance._timestamp;
        
        instance.iTOW = proto->dop(i).itow();
        instance.gDOP = proto->dop(i).gdop();
        instance.pDOP = proto->dop(i).pdop();
        instance.tDOP = proto->dop(i).tdop();
        instance.vDOP = proto->dop(i).vdop();
        instance.hDOP = proto->dop(i).hdop();
        instance.nDOP = proto->dop(i).ndop();
        instance.eDOP = proto->dop(i).edop();
        pack->dop.push(instance);
    }
    for(int i = 0; i < proto->pvt_size(); i++){
        static gps_ubx_pvt_t instance;
        static uint64_t last_timestamp = 0;

        instance._timestamp = proto->pvt(i)._inner_timestamp();
        if(instance._timestamp - last_timestamp < resample_us)
            continue;
        else
            last_timestamp = instance._timestamp;
        
        instance.iTOW = proto->pvt(i).itow();
        instance.year = proto->pvt(i).year();
        instance.month = proto->pvt(i).month();
        instance.day = proto->pvt(i).day();
        instance.hour = proto->pvt(i).hour();
        instance.min = proto->pvt(i).min();
        instance.sec = proto->pvt(i).sec();
        instance.valid = proto->pvt(i).valid();
        instance.tAcc = proto->pvt(i).tacc();
        instance.nano = proto->pvt(i).nano();
        instance.fixType = proto->pvt(i).fixtype();
        instance.flags = proto->pvt(i).flags();
        instance.flags2 = proto->pvt(i).flags2();
        instance.numSV = proto->pvt(i).numsv();
        instance.lon = proto->pvt(i).lon();
        instance.lat = proto->pvt(i).lat();
        instance.height = proto->pvt(i).height();
        instance.hMSL = proto->pvt(i).hmsl();
        instance.hAcc = proto->pvt(i).hacc();
        instance.vAcc = proto->pvt(i).vacc();
        instance.velN = proto->pvt(i).veln();
        instance.velE = proto->pvt(i).vele();
        instance.velD = proto->pvt(i).veld();
        instance.gSpeed = proto->pvt(i).gspeed();
        instance.headMot = proto->pvt(i).headmot();
        instance.sAcc = proto->pvt(i).sacc();
        instance.headAcc = proto->pvt(i).headacc();
        instance.pDOP = proto->pvt(i).pdop();
        instance.headVeh = proto->pvt(i).headveh();
        instance.magDec = proto->pvt(i).magdec();
        instance.magAcc = proto->pvt(i).magacc();
        pack->pvt.push(instance);
    }
    for(int i = 0; i < proto->hpposecef_size(); i++){
        static gps_ubx_hpposecef_t instance;
        static uint64_t last_timestamp = 0;

        instance._timestamp = proto->hpposecef(i)._inner_timestamp();
        if(instance._timestamp - last_timestamp < resample_us)
            continue;
        else
            last_timestamp = instance._timestamp;
        
        instance.version = proto->hpposecef(i).version();
        instance.iTOW = proto->hpposecef(i).itow();
        instance.ecefX = proto->hpposecef(i).ecefx();
        instance.ecefY = proto->hpposecef(i).ecefy();
        instance.ecefZ = proto->hpposecef(i).ecefz();
        instance.ecefXHp = proto->hpposecef(i).ecefxhp();
        instance.ecefYHp = proto->hpposecef(i).ecefyhp();
        instance.ecefZHp = proto->hpposecef(i).ecefzhp();
        instance.pAcc = proto->hpposecef(i).pacc();
        pack->hpposecef.push(instance);
    }
    for(int i = 0; i < proto->hpposllh_size(); i++){
        static gps_ubx_hpposllh_t instance;
        static uint64_t last_timestamp = 0;

        instance._timestamp = proto->hpposllh(i)._inner_timestamp();
        if(instance._timestamp - last_timestamp < resample_us)
            continue;
        else
            last_timestamp = instance._timestamp;
        
        instance.version = proto->hpposllh(i).version();
        instance.iTOW = proto->hpposllh(i).itow();
        instance.lon = proto->hpposllh(i).lon();
        instance.lat = proto->hpposllh(i).lat();
        instance.height = proto->hpposllh(i).height();
        instance.hMSL = proto->hpposllh(i).hmsl();
        instance.lonHp = proto->hpposllh(i).lonhp();
        instance.latHp = proto->hpposllh(i).lathp();
        instance.heightHp = proto->hpposllh(i).heighthp();
        instance.hMSLHp = proto->hpposllh(i).hmslhp();
        instance.hAcc = proto->hpposllh(i).hacc();
        instance.vAcc = proto->hpposllh(i).vacc();
        pack->hpposllh.push(instance);
    }
}


void gps_serialize_gga(gps::GGA* proto, gps_nmea_gga_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_time(data->time);
    proto->set_latitude(data->latitude);
    proto->set_north_south(std::string(data->north_south, 1));
    proto->set_longitude(data->longitude);
    proto->set_east_ovest(std::string(data->east_ovest, 1));
    proto->set_fix(data->fix);
    proto->set_satellites(data->satellites);
    proto->set_horizontal_diluition_precision(data->horizontal_diluition_precision);
    proto->set_altitude(data->altitude);
    proto->set_age_of_correction(data->age_of_correction);
}
void gps_serialize_vtg(gps::VTG* proto, gps_nmea_vtg_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_course_over_ground_degrees(data->course_over_ground_degrees);
    proto->set_course_over_ground_degrees_magnetic(data->course_over_ground_degrees_magnetic);
    proto->set_speed_kmh(data->speed_kmh);
}
void gps_serialize_gsa(gps::GSA* proto, gps_nmea_gsa_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_mode(std::string(data->mode, 1));
    proto->set_position_diluition_precision(data->position_diluition_precision);
    proto->set_horizontal_diluition_precision(data->horizontal_diluition_precision);
    proto->set_vertical_diluition_precision(data->vertical_diluition_precision);
}

void gps_serialize_dop(gps::NAV_DOP* proto, gps_ubx_dop_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_itow(data->iTOW);
    proto->set_gdop(data->gDOP);
    proto->set_pdop(data->pDOP);
    proto->set_tdop(data->tDOP);
    proto->set_vdop(data->vDOP);
    proto->set_hdop(data->hDOP);
    proto->set_ndop(data->nDOP);
    proto->set_edop(data->eDOP);
}
void gps_serialize_pvt(gps::NAV_PVT* proto, gps_ubx_pvt_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_itow(data->iTOW);
    proto->set_year(data->year);
    proto->set_month(data->month);
    proto->set_day(data->day);
    proto->set_hour(data->hour);
    proto->set_min(data->min);
    proto->set_sec(data->sec);
    proto->set_valid(data->valid);
    proto->set_tacc(data->tAcc);
    proto->set_nano(data->nano);
    proto->set_fixtype(data->fixType);
    proto->set_flags(data->flags);
    proto->set_flags2(data->flags2);
    proto->set_numsv(data->numSV);
    proto->set_lon(data->lon);
    proto->set_lat(data->lat);
    proto->set_height(data->height);
    proto->set_hmsl(data->hMSL);
    proto->set_hacc(data->hAcc);
    proto->set_vacc(data->vAcc);
    proto->set_veln(data->velN);
    proto->set_vele(data->velE);
    proto->set_veld(data->velD);
    proto->set_gspeed(data->gSpeed);
    proto->set_headmot(data->headMot);
    proto->set_sacc(data->sAcc);
    proto->set_headacc(data->headAcc);
    proto->set_pdop(data->pDOP);
    proto->set_headveh(data->headVeh);
    proto->set_magdec(data->magDec);
    proto->set_magacc(data->magAcc);
}
void gps_serialize_hpposecef(gps::NAV_HPPOSECEF* proto, gps_ubx_hpposecef_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_version(data->version);
    proto->set_itow(data->iTOW);
    proto->set_ecefx(data->ecefX);
    proto->set_ecefy(data->ecefY);
    proto->set_ecefz(data->ecefZ);
    proto->set_ecefxhp(data->ecefXHp);
    proto->set_ecefyhp(data->ecefYHp);
    proto->set_ecefzhp(data->ecefZHp);
    proto->set_pacc(data->pAcc);
}
void gps_serialize_hpposllh(gps::NAV_HPPOSLLH* proto, gps_ubx_hpposllh_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_version(data->version);
    proto->set_itow(data->iTOW);
    proto->set_lon(data->lon);
    proto->set_lat(data->lat);
    proto->set_height(data->height);
    proto->set_hmsl(data->hMSL);
    proto->set_lonhp(data->lonHp);
    proto->set_lathp(data->latHp);
    proto->set_heighthp(data->heightHp);
    proto->set_hmslhp(data->hMSLHp);
    proto->set_hacc(data->hAcc);
    proto->set_vacc(data->vAcc);
}

void gps_mapping_adaptor_construct(gps_proto_pack& pack, mapping_adaptor& mapping_map){
    mapping_map["GPS_GGA"].size = std::bind(&canlib_circular_buffer<gps_nmea_gga_t, 1000>::size, &pack.gga);
    mapping_map["GPS_GGA"].offset = std::bind(&canlib_circular_buffer<gps_nmea_gga_t, 1000>::offset, &pack.gga);
    mapping_map["GPS_GGA"].stride = sizeof(gps_nmea_gga_t);
    mapping_map["GPS_GGA"].field["_timestamp"].value._uint64 = &pack.gga.start()._timestamp;
    mapping_map["GPS_GGA"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_GGA"].field["latitude"].value._float64 = &pack.gga.start().latitude;
    mapping_map["GPS_GGA"].field["latitude"].type = mapping_type_float64;
    mapping_map["GPS_GGA"].field["longitude"].value._float64 = &pack.gga.start().longitude;
    mapping_map["GPS_GGA"].field["longitude"].type = mapping_type_float64;
    mapping_map["GPS_GGA"].field["fix"].value._uint8 = &pack.gga.start().fix;
    mapping_map["GPS_GGA"].field["fix"].type = mapping_type_uint8;
    mapping_map["GPS_GGA"].field["satellites"].value._uint8 = &pack.gga.start().satellites;
    mapping_map["GPS_GGA"].field["satellites"].type = mapping_type_uint8;
    mapping_map["GPS_GGA"].field["horizontal_diluition_precision"].value._float64  = &pack.gga.start().horizontal_diluition_precision;
    mapping_map["GPS_GGA"].field["horizontal_diluition_precision"].type = mapping_type_float64;
    mapping_map["GPS_GGA"].field["altitude"].value._float64 = &pack.gga.start().altitude;
    mapping_map["GPS_GGA"].field["altitude"].type = mapping_type_float64;
    mapping_map["GPS_GGA"].field["age_of_correction"].value._float64 = &pack.gga.start().age_of_correction;
    mapping_map["GPS_GGA"].field["age_of_correction"].type = mapping_type_float64;

    mapping_map["GPS_VTG"].size = std::bind(&canlib_circular_buffer<gps_nmea_vtg_t, 1000>::size, &pack.vtg);
    mapping_map["GPS_VTG"].offset = std::bind(&canlib_circular_buffer<gps_nmea_vtg_t, 1000>::offset, &pack.vtg);
    mapping_map["GPS_VTG"].stride = sizeof(gps_nmea_vtg_t);
    mapping_map["GPS_VTG"].field["_timestamp"].value._uint64 = &pack.vtg.start()._timestamp;
    mapping_map["GPS_VTG"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_VTG"].field["course_over_ground_degrees"].value._float64 = &pack.vtg.start().course_over_ground_degrees;
    mapping_map["GPS_VTG"].field["course_over_ground_degrees"].type = mapping_type_float64;
    mapping_map["GPS_VTG"].field["course_over_ground_degrees_magnetic"].value._float64 = &pack.vtg.start().course_over_ground_degrees_magnetic;
    mapping_map["GPS_VTG"].field["course_over_ground_degrees_magnetic"].type = mapping_type_float64;
    mapping_map["GPS_VTG"].field["speed_kmh"].value._float64 = &pack.vtg.start().speed_kmh;
    mapping_map["GPS_VTG"].field["speed_kmh"].type = mapping_type_float64;

    mapping_map["GPS_GSA"].size = std::bind(&canlib_circular_buffer<gps_nmea_gsa_t, 1000>::size, &pack.gsa);
    mapping_map["GPS_GSA"].offset = std::bind(&canlib_circular_buffer<gps_nmea_gsa_t, 1000>::offset, &pack.gsa);
    mapping_map["GPS_GSA"].stride = sizeof(gps_nmea_gsa_t);
    mapping_map["GPS_GSA"].field["_timestamp"].value._uint64 = &pack.gsa.start()._timestamp;
    mapping_map["GPS_GSA"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_GSA"].field["position_diluition_precision"].value._float64 = &pack.gsa.start().position_diluition_precision;
    mapping_map["GPS_GSA"].field["position_diluition_precision"].type = mapping_type_float64;
    mapping_map["GPS_GSA"].field["horizontal_diluition_precision"].value._float64 = &pack.gsa.start().horizontal_diluition_precision;
    mapping_map["GPS_GSA"].field["horizontal_diluition_precision"].type = mapping_type_float64;
    mapping_map["GPS_GSA"].field["vertical_diluition_precision"].value._float64 = &pack.gsa.start().vertical_diluition_precision;
    mapping_map["GPS_GSA"].field["vertical_diluition_precision"].type = mapping_type_float64;

    mapping_map["GPS_NAV_DOP"].size = std::bind(&canlib_circular_buffer<gps_ubx_dop_t, 1000>::size, &pack.dop);
    mapping_map["GPS_NAV_DOP"].offset = std::bind(&canlib_circular_buffer<gps_ubx_dop_t, 1000>::offset, &pack.dop);
    mapping_map["GPS_NAV_DOP"].stride = sizeof(gps_ubx_dop_t);
    mapping_map["GPS_NAV_DOP"].field["_timestamp"].value._uint64 = &pack.dop.start()._timestamp;
    mapping_map["GPS_NAV_DOP"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_NAV_DOP"].field["iTOW"].value._uint32 = &pack.dop.start().iTOW;
    mapping_map["GPS_NAV_DOP"].field["iTOW"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_DOP"].field["gDOP"].value._float32 = &pack.dop.start().gDOP;
    mapping_map["GPS_NAV_DOP"].field["gDOP"].type = mapping_type_float32;
    mapping_map["GPS_NAV_DOP"].field["pDOP"].value._float32 = &pack.dop.start().pDOP;
    mapping_map["GPS_NAV_DOP"].field["pDOP"].type = mapping_type_float32;
    mapping_map["GPS_NAV_DOP"].field["tDOP"].value._float32 = &pack.dop.start().tDOP;
    mapping_map["GPS_NAV_DOP"].field["tDOP"].type = mapping_type_float32;
    mapping_map["GPS_NAV_DOP"].field["vDOP"].value._float32 = &pack.dop.start().vDOP;
    mapping_map["GPS_NAV_DOP"].field["vDOP"].type = mapping_type_float32;
    mapping_map["GPS_NAV_DOP"].field["hDOP"].value._float32 = &pack.dop.start().hDOP;
    mapping_map["GPS_NAV_DOP"].field["hDOP"].type = mapping_type_float32;
    mapping_map["GPS_NAV_DOP"].field["nDOP"].value._float32 = &pack.dop.start().nDOP;
    mapping_map["GPS_NAV_DOP"].field["nDOP"].type = mapping_type_float32;
    mapping_map["GPS_NAV_DOP"].field["eDOP"].value._float32 = &pack.dop.start().eDOP;
    mapping_map["GPS_NAV_DOP"].field["eDOP"].type = mapping_type_float32;

    mapping_map["GPS_NAV_PVT"].size = std::bind(&canlib_circular_buffer<gps_ubx_pvt_t, 1000>::size, &pack.pvt);
    mapping_map["GPS_NAV_PVT"].offset = std::bind(&canlib_circular_buffer<gps_ubx_pvt_t, 1000>::offset, &pack.pvt);
    mapping_map["GPS_NAV_PVT"].stride = sizeof(gps_ubx_pvt_t);
    mapping_map["GPS_NAV_PVT"].field["_timestamp"].value._uint64  = &pack.pvt.start()._timestamp;
    mapping_map["GPS_NAV_PVT"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_NAV_PVT"].field["iTOW"].value._uint32  = &pack.pvt.start().iTOW;
    mapping_map["GPS_NAV_PVT"].field["iTOW"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_PVT"].field["year"].value._uint16  = &pack.pvt.start().year;
    mapping_map["GPS_NAV_PVT"].field["year"].type = mapping_type_uint16;
    mapping_map["GPS_NAV_PVT"].field["month"].value._uint8  = &pack.pvt.start().month;
    mapping_map["GPS_NAV_PVT"].field["month"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["day"].value._uint8  = &pack.pvt.start().day;
    mapping_map["GPS_NAV_PVT"].field["day"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["hour"].value._uint8  = &pack.pvt.start().hour;
    mapping_map["GPS_NAV_PVT"].field["hour"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["min"].value._uint8  = &pack.pvt.start().min;
    mapping_map["GPS_NAV_PVT"].field["min"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["sec"].value._uint8  = &pack.pvt.start().sec;
    mapping_map["GPS_NAV_PVT"].field["sec"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["valid"].value._uint8  = &pack.pvt.start().valid;
    mapping_map["GPS_NAV_PVT"].field["valid"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["tAcc"].value._uint32  = &pack.pvt.start().tAcc;
    mapping_map["GPS_NAV_PVT"].field["tAcc"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_PVT"].field["nano"].value._int32  = &pack.pvt.start().nano;
    mapping_map["GPS_NAV_PVT"].field["nano"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["fixType"].value._uint8  = &pack.pvt.start().fixType;
    mapping_map["GPS_NAV_PVT"].field["fixType"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["flags"].value._uint8  = &pack.pvt.start().flags;
    mapping_map["GPS_NAV_PVT"].field["flags"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["flags2"].value._uint8  = &pack.pvt.start().flags2;
    mapping_map["GPS_NAV_PVT"].field["flags2"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["numSV"].value._uint8  = &pack.pvt.start().numSV;
    mapping_map["GPS_NAV_PVT"].field["numSV"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["lon"].value._float64  = &pack.pvt.start().lon;
    mapping_map["GPS_NAV_PVT"].field["lon"].type = mapping_type_float64;
    mapping_map["GPS_NAV_PVT"].field["lat"].value._float64  = &pack.pvt.start().lat;
    mapping_map["GPS_NAV_PVT"].field["lat"].type = mapping_type_float64;
    mapping_map["GPS_NAV_PVT"].field["height"].value._int32  = &pack.pvt.start().height;
    mapping_map["GPS_NAV_PVT"].field["height"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["hMSL"].value._int32  = &pack.pvt.start().hMSL;
    mapping_map["GPS_NAV_PVT"].field["hMSL"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["hAcc"].value._uint32  = &pack.pvt.start().hAcc;
    mapping_map["GPS_NAV_PVT"].field["hAcc"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_PVT"].field["vAcc"].value._uint32  = &pack.pvt.start().vAcc;
    mapping_map["GPS_NAV_PVT"].field["vAcc"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_PVT"].field["velN"].value._int32  = &pack.pvt.start().velN;
    mapping_map["GPS_NAV_PVT"].field["velN"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["velE"].value._int32  = &pack.pvt.start().velE;
    mapping_map["GPS_NAV_PVT"].field["velE"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["velD"].value._int32  = &pack.pvt.start().velD;
    mapping_map["GPS_NAV_PVT"].field["velD"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["gSpeed"].value._int32  = &pack.pvt.start().gSpeed;
    mapping_map["GPS_NAV_PVT"].field["gSpeed"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["headMot"].value._float64  = &pack.pvt.start().headMot;
    mapping_map["GPS_NAV_PVT"].field["headMot"].type = mapping_type_float64;
    mapping_map["GPS_NAV_PVT"].field["sAcc"].value._uint32  = &pack.pvt.start().sAcc;
    mapping_map["GPS_NAV_PVT"].field["sAcc"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_PVT"].field["headAcc"].value._float64  = &pack.pvt.start().headAcc;
    mapping_map["GPS_NAV_PVT"].field["headAcc"].type = mapping_type_float64;
    mapping_map["GPS_NAV_PVT"].field["pDOP"].value._uint16  = &pack.pvt.start().pDOP;
    mapping_map["GPS_NAV_PVT"].field["pDOP"].type = mapping_type_uint16;
    mapping_map["GPS_NAV_PVT"].field["headVeh"].value._float64  = &pack.pvt.start().headVeh;
    mapping_map["GPS_NAV_PVT"].field["headVeh"].type = mapping_type_float64;
    mapping_map["GPS_NAV_PVT"].field["magDec"].value._float64  = &pack.pvt.start().magDec;
    mapping_map["GPS_NAV_PVT"].field["magDec"].type = mapping_type_float64;
    mapping_map["GPS_NAV_PVT"].field["magAcc"].value._float64  = &pack.pvt.start().magAcc;
    mapping_map["GPS_NAV_PVT"].field["magAcc"].type = mapping_type_float64;

    mapping_map["GPS_NAV_HPPOSECEF"].size = std::bind(&canlib_circular_buffer<gps_ubx_hpposecef_t, 1000>::size, &pack.hpposecef);
    mapping_map["GPS_NAV_HPPOSECEF"].offset = std::bind(&canlib_circular_buffer<gps_ubx_hpposecef_t, 1000>::offset, &pack.hpposecef);
    mapping_map["GPS_NAV_HPPOSECEF"].stride = sizeof(gps_ubx_hpposecef_t);
    mapping_map["GPS_NAV_HPPOSECEF"].field["_timestamp"].value._uint64 = &pack.hpposecef.start()._timestamp;
    mapping_map["GPS_NAV_HPPOSECEF"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_NAV_HPPOSECEF"].field["version"].value._uint8 = &pack.hpposecef.start().version;
    mapping_map["GPS_NAV_HPPOSECEF"].field["version"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_HPPOSECEF"].field["iTOW"].value._uint32 = &pack.hpposecef.start().iTOW;
    mapping_map["GPS_NAV_HPPOSECEF"].field["iTOW"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefX"].value._float64 = &pack.hpposecef.start().ecefX;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefX"].type = mapping_type_float64;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefY"].value._float64 = &pack.hpposecef.start().ecefY;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefY"].type = mapping_type_float64;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefZ"].value._float64 = &pack.hpposecef.start().ecefZ;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefZ"].type = mapping_type_float64;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefXHp"].value._int8 = &pack.hpposecef.start().ecefXHp;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefXHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefYHp"].value._int8 = &pack.hpposecef.start().ecefYHp;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefYHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefZHp"].value._int8 = &pack.hpposecef.start().ecefZHp;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefZHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSECEF"].field["pAcc"].value._float32 = &pack.hpposecef.start().pAcc;
    mapping_map["GPS_NAV_HPPOSECEF"].field["pAcc"].type = mapping_type_float32;

    mapping_map["GPS_NAV_HPPOSLLH"].size = std::bind(&canlib_circular_buffer<gps_ubx_hpposllh_t, 1000>::size, &pack.hpposllh);
    mapping_map["GPS_NAV_HPPOSLLH"].offset = std::bind(&canlib_circular_buffer<gps_ubx_hpposllh_t, 1000>::offset, &pack.hpposllh);
    mapping_map["GPS_NAV_HPPOSLLH"].stride = sizeof(gps_ubx_hpposllh_t);
    mapping_map["GPS_NAV_HPPOSLLH"].field["_timestamp"].value._uint64 = &pack.hpposllh.start()._timestamp;
    mapping_map["GPS_NAV_HPPOSLLH"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_NAV_HPPOSLLH"].field["version"].value._uint8 = &pack.hpposllh.start().version;
    mapping_map["GPS_NAV_HPPOSLLH"].field["version"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_HPPOSLLH"].field["iTOW"].value._uint32 = &pack.hpposllh.start().iTOW;
    mapping_map["GPS_NAV_HPPOSLLH"].field["iTOW"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_HPPOSLLH"].field["lon"].value._float64 = &pack.hpposllh.start().lon;
    mapping_map["GPS_NAV_HPPOSLLH"].field["lon"].type = mapping_type_float64;
    mapping_map["GPS_NAV_HPPOSLLH"].field["lat"].value._float64 = &pack.hpposllh.start().lat;
    mapping_map["GPS_NAV_HPPOSLLH"].field["lat"].type = mapping_type_float64;
    mapping_map["GPS_NAV_HPPOSLLH"].field[" height"].value._float32 = &pack.hpposllh.start(). height;
    mapping_map["GPS_NAV_HPPOSLLH"].field[" height"].type = mapping_type_float32;
    mapping_map["GPS_NAV_HPPOSLLH"].field[" hMSL"].value._float32 = &pack.hpposllh.start(). hMSL;
    mapping_map["GPS_NAV_HPPOSLLH"].field[" hMSL"].type = mapping_type_float32;
    mapping_map["GPS_NAV_HPPOSLLH"].field["lonHp"].value._int8 = &pack.hpposllh.start().lonHp;
    mapping_map["GPS_NAV_HPPOSLLH"].field["lonHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSLLH"].field["latHp"].value._int8 = &pack.hpposllh.start().latHp;
    mapping_map["GPS_NAV_HPPOSLLH"].field["latHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSLLH"].field["heightHp"].value._int8 = &pack.hpposllh.start().heightHp;
    mapping_map["GPS_NAV_HPPOSLLH"].field["heightHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSLLH"].field["hMSLHp"].value._int8 = &pack.hpposllh.start().hMSLHp;
    mapping_map["GPS_NAV_HPPOSLLH"].field["hMSLHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSLLH"].field["hAcc"].value._uint32 = &pack.hpposllh.start().hAcc;
    mapping_map["GPS_NAV_HPPOSLLH"].field["hAcc"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_HPPOSLLH"].field["vAcc"].value._uint32 = &pack.hpposllh.start().vAcc;
    mapping_map["GPS_NAV_HPPOSLLH"].field["vAcc"].type = mapping_type_uint32;
}