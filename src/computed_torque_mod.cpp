//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/computed_torque_mod.h> //library of the computed torque 
#include <ros/package.h>

namespace panda_controllers{

    // Definisco funzione oggetto thuderpanda per inizializzare il nodo di controllo
    bool ComputedTorqueMod::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
    {
        this->cvc_nh = node_handle; // cvc_nh definito con tipo ros::NodeHandle in computerd_torque.h
 
        std::string arm_id; //checking up the arm id of the robot     
        if (!node_handle.getParam("arm_id", arm_id)) {
		    ROS_ERROR("Computed Torque: Could not get parameter arm_id!");
		    return false;
        }

        /* Inizializing the Kp and Kv gains*/
    	double kp1, kp2, kp3, kv1, kv2, kv3;

        if (!node_handle.getParam("kp1", kp1) || 
		!node_handle.getParam("kp2", kp2) ||
		!node_handle.getParam("kp3", kp3) || 
		!node_handle.getParam("kv1", kv1) ||
		!node_handle.getParam("kv2", kv2) ||
		!node_handle.getParam("kv3", kv3)) {
		    ROS_ERROR("Computed Torque: Could not get parameter kpi or kv!");
		    return false;
	    }

        Kp = Eigen::MatrixXd::Identity(7, 7);
        Kp(0,0) = kp1; 
        Kp(1,1) = kp1; 
        Kp(2,2) = kp1; 
        Kp(3,3) = kp1; 
        Kp(4,4) = kp2; 
        Kp(5,5) = kp2; 
        Kp(6,6) = kp3;
        /* Assegno valori Kvi*/
        Kv = Eigen::MatrixXd::Identity(7, 7);
        Kv(0,0) = kv1; 
        Kv(1,1) = kv1; 
        Kv(2,2) = kv1; 
        Kv(3,3) = kv1; 
        Kv(4,4) = kv2; 
        Kv(5,5) = kv2; 
        Kv(6,6) = kv3;

        /* Assigning the time */
	    if (!node_handle.getParam("dt", dt)) {
		    ROS_ERROR("Computed Torque: Could not get parameter dt!");
		    return false;
	    }

        /* Chek per il acquisizione corretta del nome dei giunti del franka*/
        std::vector<std::string> joint_names;
	    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
		    ROS_ERROR("Computed Torque: Error in parsing joints name!");
		    return false;
	    }

        /* Coppia di gestione errore su corretta acquisizione e crezione di model_handel_ */
	    franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	    if (model_interface == nullptr) {
		    ROS_ERROR_STREAM("Computed Torque: Error getting model interface from hardware!");
		    return false;
	    }   
        /* Chek errori sul model_handle(necessario per usare i calcoli della dinamica del seriale)*/
        try {
		    model_handle_.reset(new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
	    } catch (hardware_interface::HardwareInterfaceException& ex) {
	    	ROS_ERROR_STREAM("Computed Torque: Exception getting model handle from interface: " << ex.what());
		    return false;
	    }
	    
        /* Coppia di gestione errore ma questa volta sulla corretta acquisizione dell'oggetto state_handle_ */
        franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    	if (state_interface == nullptr) {
	    	ROS_ERROR_STREAM("Computed Torque: Error getting state interface from hardware");
	    	return false;
	    }
        try {
		    state_handle_.reset(new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
	    } catch (hardware_interface::HardwareInterfaceException& ex) {
		    ROS_ERROR_STREAM("Computed Torque: Exception getting state handle from interface: " << ex.what());
		    return false;
	    }

        /*Gestione di non so quale errore (in tutti i seguenti errori le variabili assumono valore direttamente facendo riferimento a hardware del robot)*/
        hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
	    if (effort_joint_interface == nullptr) {
		    ROS_ERROR_STREAM("Computed Torque: Error getting effort joint interface from hardware!");
		    return false;
	    }

        /* Gestione errore sul joint_handles_ (per ogni giunto si controlla se vi è errore su trasmissione del comando) */
        for (size_t i = 0; i < 7; ++i) {
		    try {
		    	joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));

	    	} catch (const hardware_interface::HardwareInterfaceException& ex) {
		    	ROS_ERROR_STREAM("Computed Torque: Exception getting joint handles: " << ex.what());
		    	return false;
	    	}
	    }

		// - thunder init - //
		// get absolute path to franka_conf.yaml file
		std::string package_path = ros::package::getPath("panda_controllers");
		std::string path_conf = package_path + "/config/thunder/franka.yaml";
		std::string path_par_REG = package_path + "/config/thunder/franka_par_REG.yaml";
		frankaRobot.load_conf(path_conf);
		frankaRobot.load_par_REG(path_par_REG);
		param = frankaRobot.get_par_REG();
		param_frict = frankaRobot.get_par_Dl();
		// param_init = param_REG;
        
        /* Inizializing the R gains to update parameters*/
	    std::vector<double> gainRlinks(NJ), gainRparam(4);
	    Eigen::Matrix<double,PARAM,PARAM> Rlink;
        Eigen::Matrix<double,FRICTION,FRICTION> Rlink_fric;

	    if (!node_handle.getParam("gainRlinks", gainRlinks) ||
	    	!node_handle.getParam("gainRparam", gainRparam) ||
	    	!node_handle.getParam("update_param", update_param_flag)) {
	
		    ROS_ERROR("Computed_torque Could not get gain parameter for R, Kd!");
		    return false;
	    }

        /* setting della matrice di pesi R per legge di aggiornamento dei parametri */
        Rlink.setZero();
        Rlink(0,0) = gainRparam[0];
        Rlink(1,1) = gainRparam[1];
        Rlink(2,2) = Rlink(1,1);
        Rlink(3,3) = Rlink(1,1);
        Rlink(4,4) = gainRparam[2];
        Rlink(5,5) = gainRparam[3]; // Termini misti tensore di inerzia stimati meno
        Rlink(6,6) = Rlink(5,5);
        Rlink(7,7) = Rlink(4,4);
        Rlink(8,8) = Rlink(5,5);
        Rlink(9,9) = Rlink(4,4);

        Rlink_fric.setZero();
        Rlink_fric(0,0) = gainRparam[1];
        Rlink_fric(1,1) = gainRparam[2];


        /* Calcolo la matrice inversa di R utile per la legge di controllo */
        Rinv.setZero();
        Rinv_fric.setZero();
        for (int i = 0; i<NJ; i++){	
            Rinv.block(i*(PARAM), i*(PARAM), PARAM, PARAM) = gainRlinks[i]*Rlink; // block permette di fare le operazioni blocco per blocco (dubbio su che principio calcola tale inversa).
            Rinv_fric.block(i*(FRICTION), i*(FRICTION), FRICTION, FRICTION) = gainRlinks[i]*Rlink_fric; 
        }

        /* Initialize joint (torque,velocity) limits */
        tau_limit << 87, 87, 87, 87, 12, 12, 12;
        q_dot_limit << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61; 

        /*Start command subscriber and publisher */
        this->sub_command_ = node_handle.subscribe<sensor_msgs::JointState> ("/controller/command_joints", 1, &ComputedTorqueMod::setCommandCB, this);   //it verify with the callback(setCommandCB) that the command joint has been received
        this->sub_flag_update_ = node_handle.subscribe<panda_controllers::flag> ("/controller/adaptiveFlag", 1, &ComputedTorqueMod::setFlagUpdate, this);
        
        this->pub_err_ = node_handle.advertise<panda_controllers::log_adaptive_joints> ("/controller/logging", 1); //dà informazione a topic loggin l'errore che si commette 
        this->pub_config_ = node_handle.advertise<panda_controllers::point>("/controller/current_config", 1); //dà informazione sulla configurazione usata

        /* Initialize regressor object (oggetto thunderpanda) */
        // frankaRobot.init(NJ);

        return true;
    }

   
    void ComputedTorqueMod::starting(const ros::Time& time){
    
        /* Getting Robot State in order to get q_curr and dot_q_curr */
	    franka::RobotState robot_state = state_handle_->getRobotState();

        /* Getting Mass and coriolis (usando le stime attuate dal franka per i calcolo di queste?) 
        std::array<double, 49> mass_array = model_handle_->getMass();
	    std::array<double, 7> coriolis_array = model_handle_->getCoriolis(); */

        /* Mapping actual joints position, actual joints velocity, Mass matrix and Coriolis vector onto Matrix form  */
	    q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
	    dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
	  /*  M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data()); // matrice di massa calcolata usando librerie/stime del franka */
	  /* C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data()); // matrice di coriolis calcolata usando librerie/stime del franka */

        /* Secure Initialization (all'inizio il comando ai giunti corrisponde a stato attuale -> errore iniziale pari a zero) */
	    command_q_d = q_curr; // comando desiderato di posizione
        command_q_d_old = q_curr;
        command_dot_q_d = dot_q_curr; // comando desiderato di velocità
	    command_dot_q_d_old = dot_q_curr;
        command_dot_dot_q_d.setZero(); // inizialmente il comando di accelerazione si setta a zero per ogni giunto

        dot_q_curr_old = dot_q_curr;
        ddot_q_curr_old = (dot_q_curr - dot_q_curr_old) / dt;
        ddot_q_curr.setZero();
        dot_param.setZero();
        dot_param_frict.setZero();

        q_est = q_curr;
        dq_est = dot_q_curr;
        buffer_dq.push_back(dot_q_curr);

        /* Compute error (PARTE AGGIUNTA DI INIZIALIZZAZIONE)*/
        error.setZero();
	    dot_error.setZero();
        param_tot.setZero();

        dot_param.setZero();
        dot_param_frict.setZero();
        
        /* Update regressor */
        // frankaRobot.setInertialParam(param_dyn); // setta i parametri dinamici dell'oggetto frankaRobot e calcola una stima del regressore di M,C e G (che può differire da quella riportata dal franka)
        // frankaRobot.set_par_REG(param);
        frankaRobot.setArguments(q_curr, dot_q_curr, dot_q_curr, command_dot_dot_q_d); // setta i valori delle variabili di giunto di interresse e calcola il regressore Y attuale (oltre a calcolare jacobiani e simili e in maniera ridondante M,C,G)
		// auto M = frankaRobot.get_M();
		// auto C = frankaRobot.get_C();
		// auto G = frankaRobot.get_G();
		// auto Y = frankaRobot.get_Yr();
	    // std::array<double, 49> mass_array = model_handle_->getMass();
	    // std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    	// Eigen::MatrixXd M_franka = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
	    // Eigen::MatrixXd C_franka = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
	    // Eigen::MatrixXd G_franka = Eigen::Map<Eigen::Matrix<double, 7, 1>> (model_handle_->getGravity().data());
		// auto err_model = Y*param - (M*command_dot_dot_q_d + C*command_dot_q_d + G);
		// auto err_franka = Y*param - (M_franka*command_dot_dot_q_d + C_franka + G_franka);
		// std::cout << "model error: " << err_model.transpose() << std::endl<<std::endl;
		// std::cout << "franka error: " << err_franka.transpose() << std::endl<<std::endl;
    }


    void ComputedTorqueMod::update(const ros::Time&, const ros::Duration& period){
        
        franka::RobotState robot_state = state_handle_->getRobotState();
	    // std::array<double, 49> mass_array = model_handle_->getMass();
	    // std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    	// M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
	    // C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
	    G = Eigen::Map<Eigen::Matrix<double, 7, 1>> (model_handle_->getGravity().data());

		/* Actual position and velocity of the joints */
        // T0EE = Eigen::Matrix4d::Map(robot_state.O_T_EE.data()); // matrice di trasformazione omogenea che mi fa passare da s.d.r base a s.d.r EE
        q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
        dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

		/* tau_J_d is the desired link-side joint torque sensor signals "without gravity" (un concetto più teorico ideale per inseguimento dato da franka?)*/
	    tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());
        // tau_J = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J.data());

		// frankaRobot.setArguments(q_curr, dot_q_curr, dot_q_curr, command_dot_dot_q_d); // setta i valori delle variabili di giunto di interresse e calcola il regressore Y attuale (oltre a calcolare jacobiani e simili e in maniera ridondante M,C,G)
		// M = frankaRobot.get_M();
		// C = frankaRobot.get_C()*dot_q_curr;
		// G = frankaRobot.get_G();
		// auto Y = frankaRobot.get_Yr();
	    // mass_array = model_handle_->getMass();
	    // coriolis_array = model_handle_->getCoriolis();
    	// Eigen::MatrixXd M_franka = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
	    // Eigen::MatrixXd C_franka = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
	    // Eigen::MatrixXd G_franka = Eigen::Map<Eigen::Matrix<double, 7, 1>> (model_handle_->getGravity().data());
		// auto err_model = Y*param - (M*command_dot_dot_q_d + C + G);
		// auto err_franka = Y*param - (M_franka*command_dot_dot_q_d + C_franka + G_franka);
		// std::cout << "model error: " << err_model.transpose() << std::endl<<std::endl;
		// std::cout << "franka error: " << err_franka.transpose() << std::endl<<std::endl;

    	/* =============================================================================== */
        /* check matrix per vedere le stime riprodotte seguendo il calcolo del regressore*/
        /*
        frankaRobot.setArguments(q_curr,dot_q_curr,param_dyn);
        Mest = frankaRobot.get_M_gen();
        Cest = frankaRobot.get_C_gen();
        Gest = frankaRobot.get_G_gen();
        */
       /* =============================================================================== */
    
        // q_est_old = q_est;
        // q_est = q_est + 0.9*(q_curr - q_est);
        // dot_q_curr = (q_curr - q_est_old)/dt;

		// ----- Filters ----- //
		// Filtro velocità e accelerazioni dopo calcolo errore
        addValue(buffer_dq, dot_q_curr, WIN_LEN);
        dq_est = obtainMean(buffer_dq);
		ddot_q_curr = (dq_est - dot_q_curr_old)/dt;
		// addValue(buffer_ddq, ddot_q_curr, WIN_LEN);
        // ddot_q_curr = obtainMean(buffer_ddq);
        dot_q_curr_old = dq_est;

        // dq_est.setZero();
        // ddot_q_curr_old.setZero();
        // Filro semplice per ddot_q_curr(tipo feeding filter)
        // ddot_q_curr = 0.8*ddot_q_curr + 0.2*ddot_q_curr_old;
        // ddot_q_curr_old = ddot_q_curr;

        /* Saturate desired velocity to avoid limits*/  
        // for (int i = 0; i < 7; ++i){
		// double ith_des_vel = std::fabs(command_dot_q_d(i)/q_dot_limit(i));
		//     if( ith_des_vel > 1){
		//         command_dot_q_d = command_dot_q_d / ith_des_vel; 
        //     }    
	    // }

        error = command_q_d - q_curr; // errore di posizione(posizione desiderata - quella reale)
	    dot_error = command_dot_q_d - dot_q_curr; // errore di velocità ai giunti 
	    // x.head(7) = error; // x indica la variabile di stato composta da [error dot_error]'
	    // x.tail(7) = dot_error;

     /*   // Publish tracking errors as joint states(ad ogni update nodo mi mostra il valore di errore di posizione e velocità ai giunti e l'istante di riferimento)
        sensor_msgs::JointState error_msg;
        std::vector<double> err_vec(error.data(), error.data() + error.rows()*error.cols());
        std::vector<double> dot_err_vec(dot_error.data(), dot_error.data() + dot_error.rows()*dot_error.cols());
        
        error_msg.header.stamp = ros::Time::now();
        error_msg.position = err_vec;
        error_msg.velocity = dot_err_vec;
        this->pub_err_.publish(error_msg);*/

        /* Sempre stessi valori dei guadagni*/
    	// Kp = Kp;
    	// Kv = Kv;

        /*Compute Friction matrix before filter*/
        Dest.setZero();
        for(int i = 0; i < 7; ++i){
            // Dest(i,i) = param_frict((FRICTION)*i,0) + param_frict((FRICTION)*i+1,0)*fabs(dot_q_curr(i));
            Dest(i,i) = param_frict((FRICTION)*i,0) + param_frict((FRICTION)*i+1,0)*deltaCompute(dot_q_curr(i));
        }

        /* Update and Compute Regressor mod e Regressor Classic*/
	    frankaRobot.setArguments(q_curr, dot_q_curr, command_dot_q_d, command_dot_dot_q_d);
	    Y_mod = frankaRobot.get_Yr(); // calcolo del regressore
        frankaRobot.setArguments(q_curr, dot_q_curr, dot_q_curr, ddot_q_curr);
        Y_norm = frankaRobot.get_Yr();
        // ROS_INFO_STREAM(Y_norm.transpose());
        // redY_norm = Y_norm.block(0,(NJ-1)*PARAM,NJ,PARAM);

        /*Calcolo a meno del regressore di attrito*/
        Y_D.setZero();
        Y_D_norm.setZero();
        for (int i = 0; i < 7; ++i) {
            Y_D(i, i * 2) = command_dot_q_d(i); // Imposta 1 sulla diagonale principale
            // Y_D(i, i * 2 + 1) = command_dot_q_d(i)*fabs(dot_q_curr(i)); // Imposta q_i sulla colonna successiva alla diagonale
            Y_D(i, i * 2 + 1) = command_dot_q_d(i)*deltaCompute(dot_q_curr(i));
            Y_D_norm(i, i * 2) = dot_q_curr(i); // Imposta 1 sulla diagonale principale
            // Y_D_norm(i, i * 2 + 1) = dot_q_curr(i)*fabs(dot_q_curr(i)); // Imposta q_i sulla colonna successiva alla diagonale
            Y_D_norm(i, i * 2 + 1) = dot_q_curr(i)*deltaCompute(dot_q_curr(i));
        }

        // ROS_INFO_STREAM("Y_D:" << Y_D << "Dest:" << Dest);

        /*Gravità compensata non va nel calcolo del residuo*/
        tau_J = tau_J_d + G;
        // addValue(buffer_tau, tau_J, WIN_LEN);

        // Media dei dati nella finestra del filtro
        // tau_J = obtainMean(buffer_tau);
        // Y_mod_D << Y_mod, Y_D; // concatenation
        // Y_norm_D << Y_norm, Y_D; // concatenation
        
        err_param = tau_J - Y_norm*param; // - Y_D_norm*param_frict;
        // param7 = param.segment((NJ-1)*PARAM, PARAM);

        /* se vi è stato aggiornamento, calcolo il nuovo valore che paramatri assumono secondo la seguente legge*/
        if (update_param_flag){
            dot_param = 0.01*Rinv*(Y_mod.transpose()*dot_error + 0.3*Y_norm.transpose()*(err_param));
            param = param + dt*dot_param;
            // dot_param_frict = 0.01*Rinv_fric*(Y_D.transpose()*dot_error + 0.3*Y_D_norm.transpose()*(err_param));
            // param_frict = param_frict + dt*dot_param_frict;
	    }


        /*Riordino parametri per ogni link*/
        for(int i = 0; i < 7; ++i){
            param_tot.segment(i*(PARAM+FRICTION),PARAM) = param.segment(i*(PARAM),PARAM);
            param_tot.segment(i*(PARAM+FRICTION) + PARAM,FRICTION) = param_frict.segment(i*FRICTION,FRICTION);
        }
        // ROS_INFO_STREAM(param_tot);

        /* update dynamic for control law */
        frankaRobot.set_par_REG(param);
        Mest = frankaRobot.get_M();
        Cest = frankaRobot.get_C();
        Gest = frankaRobot.get_G();

        /* command torque to joint */
        tau_cmd = Mest * command_dot_dot_q_d + Cest * command_dot_q_d  + Kp * error + Kv * dot_error + Gest;

	    // /* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */
	    tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d+G);

        /* Set the command for each joint */
	    for (size_t i = 0; i < 7; i++) {
	    	joint_handles_[i].setCommand(tau_cmd[i]-G[i]);
	    }

        /* Publish messages*/
		Eigen::Map<Eigen::MatrixXd> Kp_vect(Kp.data(), 49,1);
		Eigen::Map<Eigen::MatrixXd> Kv_vect(Kv.data(), 49,1);

        time_now = ros::Time::now();
        msg_log.header.stamp = time_now;

        fillMsg(msg_log.error_q, error);
        fillMsg(msg_log.dot_error_q, dot_error);
        fillMsg(msg_log.q_cur, q_curr);
		fillMsg(msg_log.dot_q_cur, dot_q_curr);
        fillMsgLink(msg_log.link1, param_tot.segment(0, PARAM+FRICTION));
        fillMsgLink(msg_log.link2, param_tot.segment(12, PARAM+FRICTION));
        fillMsgLink(msg_log.link3, param_tot.segment(24, PARAM+FRICTION));
        fillMsgLink(msg_log.link4, param_tot.segment(36, PARAM+FRICTION));
        fillMsgLink(msg_log.link5, param_tot.segment(48, PARAM+FRICTION));
        fillMsgLink(msg_log.link6, param_tot.segment(60, PARAM+FRICTION));
        fillMsgLink(msg_log.link7, param_tot.segment(72, PARAM+FRICTION));
        fillMsg(msg_log.tau_cmd, tau_cmd);
        fillMsg(msg_log.ddot_q_curr, ddot_q_curr);
		fillMsg(msg_log.Kp, Kp_vect);
		fillMsg(msg_log.Kv, Kv_vect);

        msg_config.header.stamp  = time_now;
        msg_config.xyz.x = T0EE.translation()(0); 
        msg_config.xyz.y = T0EE.translation()(1);
        msg_config.xyz.z = T0EE.translation()(2);

        this->pub_err_.publish(msg_log);
        this->pub_config_.publish(msg_config);
    }

    void ComputedTorqueMod::stopping(const ros::Time&)
    {
	//TO DO
    }

    // add value to buffer
    void ComputedTorqueMod::addValue(std::vector<Eigen::Matrix<double,7, 1>>& buffer_, const Eigen::Matrix<double,7, 1>& dato_, int win_len) {
        buffer_.push_back(dato_);
        if (buffer_.size() > win_len) {
            buffer_.erase(buffer_.begin());
        }
    }

    // compute mean from buffer
    Eigen::Matrix<double,7, 1> ComputedTorqueMod::obtainMean(const std::vector<Eigen::Matrix<double,7, 1>>& buffer_) {
        Eigen::Matrix<double,7, 1> mean = Eigen::Matrix<double,7, 1>::Zero();
        for (const auto& vector : buffer_) {
            mean += vector;
        }
        mean /= buffer_.size();
        return mean;
    }

    double ComputedTorqueMod::deltaCompute (double a){
        double delta;
        
        if (fabs(a) < 0.01){
            delta = 0.0; 
        }else{
            delta = 1/fabs(a);
        }
        return delta;
    }

    /* Check for the effort commanded */
    Eigen::Matrix<double, 7, 1> ComputedTorqueMod::saturateTorqueRate(
	const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
	const Eigen::Matrix<double, 7, 1>& tau_J_d)
    {
	    Eigen::Matrix<double, 7, 1> tau_d_saturated;
	    for (size_t i = 0; i < 7; i++) {

		    double difference = tau_d_calculated[i] - tau_J_d[i];
		    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);

	    }
	    return tau_d_saturated;
    }

    void ComputedTorqueMod::setCommandCB(const sensor_msgs::JointStateConstPtr& msg)
    {
        command_dot_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->effort).data());
        command_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->velocity).data());
        command_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
        // command_dot_q_d = command_dot_q_d + dt*command_dot_dot_q_d;
        // command_q_d = command_q_d + dt*command_dot_q_d;

    }

    void ComputedTorqueMod::setFlagUpdate(const panda_controllers::flag::ConstPtr& msg){
        update_param_flag = msg->flag;
    }


    template <size_t N>
    void ComputedTorqueMod::fillMsg(boost::array<double, N>& msg_, const Eigen::VectorXd& data_) {
        int dim = data_.size();
        for (int i = 0; i < dim; i++) {
            msg_[i] = data_[i];
        }
    }

    void ComputedTorqueMod::fillMsgLink(panda_controllers::link_params &msg_, const Eigen::VectorXd& data_) {      
        msg_.mass = data_[0];
        msg_.m_CoM_x = data_[1];
        msg_.m_CoM_y = data_[2];
        msg_.m_CoM_z = data_[3];
        msg_.Ixx = data_[4];
        msg_.Ixy = data_[5];
        msg_.Ixz = data_[6];
        msg_.Iyy = data_[7];
        msg_.Iyz = data_[8];
        msg_.Izz = data_[9];
        msg_.d1 = data_[10];
        msg_.d2 = data_[11];
    }

}
PLUGINLIB_EXPORT_CLASS(panda_controllers::ComputedTorqueMod, controller_interface::ControllerBase);