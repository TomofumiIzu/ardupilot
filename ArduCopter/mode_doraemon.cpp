#include "Copter.h"
#include <utility>
#include <regex>

/*
 * Init and run calls for doraemon flight mode
 */

// doraemon_run - runs the main stabilize controller
// should be called at 100hz or more

static Vector3p doraemon_pos_target_cm;       // position target (used by posvel controller only)
bool doraemon_pos_terrain_alt;                // true if guided_pos_target_cm.z is an alt above terrain
static Vector3f doraemon_vel_target_cms;      // velocity target (used by pos_vel_accel controller and vel_accel controller)
static Vector3f doraemon_accel_target_cmss;   // acceleration target (used by pos_vel_accel controller vel_accel controller and accel controller)
static uint32_t doraemon_update_time_ms;             // system time of last target update to pos_vel_accel, vel_accel or accel controller

bool ModeDoraemon::init(bool ignore_checks)
{
 
    //ファイルのオープン
    if ((d_file = fopen("./scripts/object.json", "r")) == NULL){
        gcs().send_text(MAV_SEVERITY_ERROR, "not exists [object.json]");
        return (opn_flg = false);
    }else{
        opn_flg = true;
    }
    
    pva_control_start();
    doraemon_vel_target_cms.zero();
    doraemon_accel_target_cmss.zero();
    end_flg = false;
    return true;
}


void ModeDoraemon::run()
{
    if (!copter.arming.is_armed()){
        d_mode = SubMode::land;
        d_takeoff_alt_cm = 0.0;
        return ;
    }
    
    copter.set_auto_armed(true);
    if (d_takeoff_alt_cm > 0.0 && d_mode == SubMode::TakeOff){
        auto_takeoff_start(d_takeoff_alt_cm, false);
        auto_takeoff_run();
        Vector3p takeoff_npos;
        if (auto_takeoff_get_position(takeoff_npos)){
            d_mode = SubMode::WP;
        }
    }else if (d_mode == SubMode::WP) {
        char buff[buff_size + 1];
        std::string content = "";

        for (int i=0; i <= buff_size; i++){
            buff[i] = '\0';
        }
        
        vp.clear();
        if (fread(buff, buff_size ,1, d_file) == 0){
            fclose(d_file);
            end_flg = true;
        }
        content = save_str + std::string(buff);
 

        std::string searchStr = "(-|[0-9]){1,4}\\.([0-9])*" ;

        // 正規表現の検索パターン用オブジェクトを生成
        std::regex searchPattern( searchStr ) ;
        std::match_results<const char *> matchResult ;

        uint32_t buff_cnt;
        std::string matchedStr;
        std::string b_matchedStr;
        int cnt = 0;
        int vp_cnt = 0;
        while (true){
            // マッチング
            bool isMatched = std::regex_search( content.c_str(), matchResult, searchPattern ) ;
            if(!isMatched){
                break ;
            }
            cnt++;
            buff_cnt = (uint32_t)matchResult.position();
            matchedStr = matchResult.str();
            if (cnt % 2 == 0){
                int32_t land_lat; int32_t land_lng;
                if (vp_cnt == 0){
                    Location loc;
                    ahrs.get_location(loc);
                    land_lat = loc.lat;
                    land_lng = loc.lng;
                }else{
                    land_lat = vp[vp_cnt - 1].first;
                    land_lng = vp[vp_cnt - 1].second;
                }
                int32_t s_lat; int32_t s_lng;
                //gcs().send_text(MAV_SEVERITY_DEBUG, "s_lat:%s, s_lng:%s", matchedStr.c_str(), b_matchedStr.c_str());
                s_lat = get_latlng_ardu(matchedStr.c_str());
                s_lng = get_latlng_ardu(b_matchedStr.c_str());
                if (judge_target(land_lat, land_lng, s_lat, s_lng, 1) > 0){
                    vp.push_back(Pair(s_lat, s_lng));
                    vp_cnt++;
                }
                if ((size_t)wrap_byte >= content.length()){
                    
                    save_str = content.substr(buff_cnt + matchedStr.length());
                    //gcs().send_text(MAV_SEVERITY_DEBUG, "save_str: [%s]", save_str.c_str());
                    break;
                }

            }else{
                b_matchedStr = matchedStr;
            }
            content = content.substr(buff_cnt + matchedStr.length());
        }
        step = 0;
        d_mode = SubMode::VelAccel;
    } else if (d_mode == SubMode::VelAccel){
        //gcs().send_text(MAV_SEVERITY_DEBUG, "VelAccel");
        if (vp.empty() && end_flg){
            d_mode = SubMode::RTL;
            return ;
        }


        Location loc;
        Location target = Location();

        target.lat = vp[step].first;
        target.lng = vp[step].second;
        if (ahrs.get_location(loc)) {
            float dist = judge_target(loc.lat, loc.lng, target.lat, target.lng, 2);
            if (!(dist > 0.0)){
                if (step == vp.size() - 1 && !end_flg){
                    d_mode = SubMode::WP;
                    return ;
                } else if (step == vp.size() - 1 && end_flg){
                    d_mode = SubMode::RTL;
                    return ;
                }
                step++;
                target.lat = vp[step].first;
                target.lng = vp[step].second;
                dist = judge_target(loc.lat, loc.lng, target.lat, target.lng, 2);
            }
            float dec_speed = dist;
            if (speed < dist && dist <= 30 ){
                dec_speed = speed;
            }

            Vector3f vel_ned = Vector3f(); 
            Vector3f pos = loc.get_distance_NED(target);

            vel_ned.x = (pos.x *  dec_speed / (sqrtf(pos.x * pos.x + pos.y * pos.y)));
            vel_ned.y = (pos.y * dec_speed / (sqrtf(pos.x * pos.x + pos.y * pos.y)));

            float alt_diff = 0.0;
            AP_Terrain *terrain = AP::terrain();
            terrain->height_above_terrain(alt_diff, true);
            if (alt_diff < 10){
                vel_ned.z = speed;
            }else{
                vel_ned.z = 0.0;
            }
            const Vector3f vel_neu_cms(vel_ned.x * 100.0f, vel_ned.y * 100.0f, vel_ned.z * 100.0f);
            set_velocity(vel_neu_cms);
            velaccel_control_run();
        }
    } else if (d_mode == SubMode::RTL){
        if (!copter.set_mode(Mode::Number::RTL, ModeReason::AUTOROTATION_BAILOUT)) {
            copter.set_mode(Mode::Number::LAND, ModeReason::AUTOROTATION_BAILOUT);
        }
    } 

}

// set_velocity - sets guided mode's target velocity
void ModeDoraemon::set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{
    set_velaccel(velocity, Vector3f(), use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw, log_request);
}

// set_velaccel - sets guided mode's target velocity and acceleration
void ModeDoraemon::set_velaccel(const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{

    if (d_mode != SubMode::VelAccel){
        d_mode = SubMode::VelAccel;
        pva_control_start();
    }
    

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);
    //auto_yaw.set_mode_to_default(false);
    
    // set velocity and acceleration targets and zero position
    doraemon_pos_target_cm.zero();
    doraemon_pos_terrain_alt = false;
    doraemon_vel_target_cms = velocity;
    //gcs().send_text(MAV_SEVERITY_DEBUG, "1.target x:%f y:%f z:%f", doraemon_vel_target_cms.x, doraemon_vel_target_cms.y, doraemon_vel_target_cms.z);

    doraemon_accel_target_cmss = acceleration;
    doraemon_update_time_ms = millis();

    // log target
    if (log_request) {
      copter.Log_Write_Guided_Position_Target(ModeGuided::SubMode::VelAccel, doraemon_pos_target_cm.tofloat(), doraemon_pos_terrain_alt, doraemon_vel_target_cms, doraemon_accel_target_cmss);
    }
}
bool ModeDoraemon::has_user_takeoff(bool must_navigate) const {
    return true;
}

bool ModeDoraemon::do_user_takeoff_start(float takeoff_alt_cm)
{
    d_takeoff_alt_cm = takeoff_alt_cm;
    d_mode = SubMode::TakeOff;

    return true;
}


// initialise position controller
void ModeDoraemon::pva_control_start()
{
    
    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise velocity controller
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    // initialise yaw
    auto_yaw.set_mode_to_default(false);

    // initialise terrain alt
    doraemon_pos_terrain_alt = false;
}


// velaccel_control_run - runs the guided velocity and acceleration controller
// called from guided_run
void ModeDoraemon::velaccel_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }
    

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    
    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - doraemon_update_time_ms > get_timeout_ms()) {
        doraemon_vel_target_cms.zero();
        doraemon_accel_target_cmss.zero();
        if ((auto_yaw.mode() == AUTO_YAW_RATE) || (auto_yaw.mode() == AUTO_YAW_ANGLE_RATE)) {
            auto_yaw.set_rate(0.0f);
        }
    }
    

    bool do_avoid = false;
#if AC_AVOID_ENABLED
    // limit the velocity for obstacle/fence avoidance
    //gcs().send_text(MAV_SEVERITY_DEBUG, "2.target x:%f y:%f z:%f", doraemon_vel_target_cms.x, doraemon_vel_target_cms.y, doraemon_vel_target_cms.z);

    copter.avoid.adjust_velocity(doraemon_vel_target_cms, pos_control->get_pos_xy_p().kP(), pos_control->get_max_accel_xy_cmss(), pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), G_Dt);
    do_avoid = copter.avoid.limits_active();
#endif

    
    // update position controller with new target
    if (!stabilizing_vel_xy() && !do_avoid) {
        // set the current commanded xy vel to the desired vel
        doraemon_vel_target_cms.x = pos_control->get_vel_desired_cms().x;
        doraemon_vel_target_cms.y = pos_control->get_vel_desired_cms().y;
    }
    pos_control->input_vel_accel_xy(doraemon_vel_target_cms.xy(), doraemon_accel_target_cmss.xy(), false);
    
    if (!stabilizing_vel_xy() && !do_avoid) {
        // set position and velocity errors to zero
        pos_control->stop_vel_xy_stabilisation();
    } else if (!stabilizing_pos_xy() && !do_avoid) {
        // set position errors to zero
        pos_control->stop_pos_xy_stabilisation();
    }
    
    pos_control->input_vel_accel_z(doraemon_vel_target_cms.z, doraemon_accel_target_cmss.z, false, false);

    // call velocity controller which includes z axis controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    //gcs().send_text(MAV_SEVERITY_DEBUG, "4.target x:%f y:%f z:%f", pos_control->get_pos_target_cm().x, pos_control->get_pos_target_cm().y, pos_control->get_pos_target_z_cm());


    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from position controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from position controller, yaw rate from mavlink command or mission item
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), auto_yaw.rate_cds());
    } else {
        // roll & pitch from position controller, yaw heading from GCS or auto_heading()
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.yaw(), auto_yaw.rate_cds());
    }
}


// return guided mode timeout in milliseconds. Only used for velocity, acceleration, angle control, and angular rates
uint32_t ModeDoraemon::get_timeout_ms() const
{
    return MAX(copter.g2.guided_timeout, 0.1) * 1000;
}
// returns true if GUIDED_OPTIONS param specifies velocity should  be controlled (when acceleration control is active)
bool ModeDoraemon::stabilizing_vel_xy() const
{
    return !((copter.g2.guided_options.get() & uint32_t(Options::DoNotStabilizeVelocityXY)) != 0);
}
// returns true if GUIDED_OPTIONS param specifies position should be controlled (when velocity and/or acceleration control is active)
bool ModeDoraemon::stabilizing_pos_xy() const
{
    return !((copter.g2.guided_options.get() & uint32_t(Options::DoNotStabilizePositionXY)) != 0);
}

// helper function to set yaw state and targets
void ModeDoraemon::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw && relative_angle) {
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    } else if (use_yaw && use_yaw_rate) {
        auto_yaw.set_yaw_angle_rate(yaw_cd * 0.01f, yaw_rate_cds * 0.01f);
    } else if (use_yaw && !use_yaw_rate) {
        auto_yaw.set_yaw_angle_rate(yaw_cd * 0.01f, 0.0f);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate(yaw_rate_cds);
    } else {
        auto_yaw.set_mode_to_default(false);
    }
}

float ModeDoraemon::judge_target(int32_t lat, int32_t lng, int32_t n_lat, int32_t n_lng, int mode){
    Location now = Location();
    Location next = Location();
    float val;

    if (mode == 1){
        val = 2.0;
    }else{
        val = 0.5;
    }

    now.lat = lat;
    now.lng = lng;
    next.lat = n_lat;
    next.lng = n_lng;

    float dist_m = now.get_distance(next);

    if (dist_m > val){
        return dist_m;
    }
    return 0.0;
}

int32_t get_latlng_ardu(const char* num_chars, uint32_t num_digits){
    bool judge = false;
    uint32_t i = 0;
    uint32_t d_i = 0;
    std::string str = "";
    std::string strnum = std::string(num_chars);
    int val = 0;

    if (strnum.length() == 0){
        return 0;
    }
    while (true){
        if (i < strnum.length() ){
            if (strnum[i] == '.'){
                judge = true;
                i++;
                continue ;
            } else {
                if (d_i < num_digits) { //小数点桁数が指定桁未満の場合
                    str += strnum[i];
                } else {
                    val = strnum[i] - 0x30; //小数点桁数が指定桁の次の場合は四捨五入の判断のため変数に配置
                    break ;
                }
            }
        } else {
            judge = true;
            if (d_i < num_digits) {
                str += "0";
            } else {
                break ;
            }
        }
        if (judge){
             d_i++; 
        }
        i++;
    }
    int32_t ret = atoi(str.c_str());
    if (val > 4) {
        ret++;
    }
    return ret;
}