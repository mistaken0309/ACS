/**
* Name: common_agents
* Author: Annalisa Congiu
* Description: agents common between centralised and decentralised simulations
* Tags: 
*/


model common_agents

import "./../models/simulationAVmain.gaml"

/* Species common to both centralised and decentralised simulation.
 * Precisely:
 * 	cars: also parent for the centralised and decentralised different species of autonomous vehicles;
 * 	people: agents representing people simulated;
 * 	intersection: agents simulating intersections; **
 * 	road: agents representing roads in the system; **
 * 	building: agents representing buildings in the systems. **
 * 		** created from osm data elaborate via create_maps.gaml
 */


//species that will represent the intersection node, it can be traffic lights or not, using the skill_road_node skill
species intersection skills: [skill_road_node] {
	bool is_traffic_signal;
	list<list> stop;
	int time_to_change <- rnd(12,20);
	int counter <- rnd (time_to_change) ;
	list<road> ways1;
	list<road> ways2;
	bool is_green;
	rgb color_fire;
	
	action initialize {
		if (is_traffic_signal) {
			do compute_crossing;
			stop<< [];
			if (flip(0.5)) {
				do to_green;
			} else {
				do to_red;
			}	
		}
	}
	
	action compute_crossing{
		if  (length(roads_in) >= 2) {
			road rd0 <- road(roads_in[0]);
			list<point> pts <- rd0.shape.points;						
			float ref_angle <-  float( last(pts) direction_to rd0.location);
			loop rd over: roads_in {
				list<point> pts2 <- road(rd).shape.points;						
				float angle_dest <-  float( last(pts2) direction_to rd.location);
				float ang <- abs(angle_dest - ref_angle);
				if (ang > 45 and ang < 135) or  (ang > 225 and ang < 315) {
					ways2<< road(rd);
				}
			}
		}
		loop rd over: roads_in {
			if not(rd in ways2) {
				ways1 << road(rd);
			}
		}
	}
	
	action to_green {
		stop[0] <- ways2 ;
		color_fire <- #green;
		is_green <- true;
	}
	
	action to_red {
		stop[0] <- ways1;
		color_fire <- #red;
		is_green <- false;
	}
	
	reflex dynamic_node when: is_traffic_signal  {
		counter <- counter + 1;
		if (counter >= time_to_change) { 
			counter <- 0;
			if is_green {do to_red;}
			else {do to_green;}
		} 
	}
	
	aspect base {
		if (is_traffic_signal) {	
			draw circle(5) color: color_fire;
		}
	}
	
	aspect base3D {
		if (is_traffic_signal) {	
			draw box(1,1,10) color:#black;
			draw sphere(5) at: {location.x,location.y,12} color: color_fire;
		}
	}
}

//species that will represent the roads
species road skills: [skill_road] { 
	geometry geom_display;
	string oneway;
	string junction;
	float perim<-shape.perimeter;	
	int nb_agents<-length(all_agents) update: length(all_agents);
	float capacity <- 1+(shape.perimeter*lanes)/v_length;
	float speed_coeff<- 0.1 update: (length(all_agents)/capacity) min:0.1 max:((shape.perimeter < 1.0) ? 1.0 : 3.0);
	
	int ov_rgb<-150 update: 150-(150*int(speed_coeff-0.1)) min: 0 max:255; //0.1 ->150 // 1 e oltre -> 0
	int ov_rgbR<-150 update: 255*int(speed_coeff-0.1)  min: 150 max: 255; // 1 e oltre -> 255 // 0.1 -> 0
	rgb color<-rgb(127,127,127) update: rgb(ov_rgbR, ov_rgb, ov_rgb);
	
	aspect base {    
		draw shape color: color end_arrow: 6;
	} 
	aspect base3D {    
		draw geom_display color: #gray ;
	} 
}

//species that will represent the buildings
species building{
	string type;
	string group;
	rgb color<- rgb(200,200,200);

	aspect base{
		draw shape color: color border: #gray;
	}
}


//Car species (ensemble) that will move on the graph of roads to a target and using the skill advanced_driving
//parent of the species representing the autonomous vehicles
species cars skills: [advanced_driving]{ 

	rgb color <- rgb(rnd(255), rnd(255), rnd(255)) ;
	int counter_stucked <- 0;
	int threshold_stucked;
	bool breakdown <- false;
	float proba_breakdown ;
	intersection the_target;

	reflex breakdown when: flip(proba_breakdown){
		breakdown <- true;
		max_speed <- 1 °km/°h;
	}
	reflex keep_in_check_real_speed{
//		real_speed <- road(current_road).maxspeed;
		float rs_ma<-(real_speed + max_acceleration);
		float road_max;
		float confront_car_road ;
		if current_road!=nil{
			road_max <- (road(current_road).maxspeed * speed_coeff);
			confront_car_road <- (rs_ma < road_max ? rs_ma: road_max);
		}else{
			confront_car_road <- rs_ma;
		}
		real_speed <- (max_speed<confront_car_road) ? max_speed : confront_car_road;
	}
	
	reflex time_to_go when: final_target = nil {
		the_target <- one_of(intersection where not each.is_traffic_signal);
		current_path <- compute_path(graph: road_network, target: the_target);
		if (current_path = nil ) {
			final_target <- nil;
		}
	}
	reflex move when: current_path != nil and final_target != nil {
		do drive;
		if real_speed < 5°km/°h {
			counter_stucked<- counter_stucked + 1;
			if (counter_stucked mod threshold_stucked = 0) {
				proba_use_linked_road <- min([1.0,proba_use_linked_road + 0.1]);
			}
		} else {
			counter_stucked<- 0;
			proba_use_linked_road <- 0.0;
		}
	}
	
	aspect base { 
		draw image_file("../includes/car-png-top-white-top-car-png-image-34867-587.png") size:{5#m,2.5#m} rotate: heading+180;
		draw breakdown ? square(15) : triangle(15) color: color rotate:heading + 90;
	} 
	aspect base3D {
		point loc <- calcul_loc();
		draw box(vehicle_length, 1,1) at: loc rotate:  heading color: color;
		
		draw triangle(0.5) depth: 1.5 at: loc rotate:  heading + 90 color: color;
		
		if (breakdown) {
			draw circle(2) at: loc color: #red;
		}
	} 
	
	point calcul_loc {
		if (current_road = nil) {
			return location;
		} else {
			float val <- (road(current_road).lanes - current_lane) + 0.5;
			val <- on_linked_road ? val * - 1 : val;
			if (val = 0) {
				return location; 
			} else {
				return (location + {cos(heading + 90) * val, sin(heading + 90) * val});
			}
		}
	}
} 

//species representing a simulation of people that are looking for a lift to {go to/come back from} work 
species people skills:[moving] control: fsm {
	rgb color <- #royalblue;
	building living_place <- nil ;
	building working_place <- nil ;
	float start_work ;
	float end_work ;
	point the_target <- nil ;
	float dist<-0.0 ;
	float dist_covered_alone ;
	intersection origin;
	
	bool starting<-true;
	bool late<-false;
	float actual_time_in;
	
	map road_knowledge<-graph_weights update: graph_weights;
	path path_to_follow<-nil;
	float look_up<-50.0;
	string next_state<-nil;
	bool got_lift<-false; 
	list<intersection> close_intersections <- nil 
		update: ((state contains 'search_lift') ) ? intersection overlapping (self.shape+look_up) : nil;
	
	float distance_to_cover;
	float time_to_cover<-0.0;
	float time_needed<-0.0;
	float cost_to_cover<-0.0;
	float cost_proposed<-0.0;
	float departure_time<-0.0;
	float arrival_time<-0.0;
	float waiting_time<-0.0 update: ((state contains "search_lift") and got_lift) ? (current_hour - waiting_time) : waiting_time;
	bool set_waiting_time<-true;
	float time_trip <- 0.0;
	float total_time_needed  <- 0.0;
	
//	STATES OF THE passengers FSM
	
	state resting initial:true{
		enter{
			color <- #lightblue;
			next_state<-'search_lift_work';
		}
		transition to: search_lift_work when: current_hour = start_work-before_work_search;
	}
	state search_lift_work{
		enter{
			the_target <- working_place.location;
			color<- #yellow;
			late<-false;
			next_state<-'go_work';
		}
		transition to: go_work when: current_hour = start_work -before_work_start;
	}
	state wait_for_lift{
		if current_hour>start_work and current_hour<end_work{
			late<-true;
		}else{
			late<-false;
		}
		color<-#darkseagreen;
	}
	state go_work{
		enter{
			the_target<-working_place.location;
			if current_hour>start_work{
				late<-true;
			} else{
				late<-false;
			}
			color<-#thistle;
			next_state<-'working';
		}
		transition to: working when: current_hour >= start_work and self.location = working_place.location;
	}
	state working{
		enter{
			color <- #blue;
			actual_time_in<-time/3600;
			if late{write string(self.name) + " ATI "+actual_time_in + " SUPPOSED "+start_work ;}
			next_state<-'search_lift_home';
		}
		transition to: search_lift_home when: current_hour = end_work;
	}
	state search_lift_home{
		enter{
			the_target <- living_place.location;
			color<- #yellow;	
			late<-false;
			color<-#thistle;
			next_state<-'go_home';
		}
		transition to: go_home when: current_hour = end_work+after_work_start;
	}
	state go_home{
		enter{
			the_target<-living_place.location;
			next_state<-'resting';
		}
		transition to: resting when: self.location = living_place.location;		
	}
	
	reflex start_waiting_time when: (state contains "search_lift") and got_lift=false and set_waiting_time{
		if set_waiting_time{
			waiting_time <- current_hour;
			set_waiting_time<-false;
		}
	}
	
//	the agent computes the path to reach its destination
	reflex search_path when: the_target!=nil and path_to_follow=nil and ((state contains "search_lift") or 
		(state contains 'go' and got_lift=true and location!=the_target)){
		if (path_to_follow = nil) {
			//Find the shortest path using the agent's own weights to compute the shortest path
			path_to_follow <- path_between(the_graph with_weights road_knowledge, location,the_target);
			if path_to_follow!=nil{
				list<geometry> segments <- path_to_follow.segments;
				loop seg over:segments{
					dist <- (dist + seg.perimeter );
					time_needed<- (time_needed + (seg.perimeter/(speed)));
					time_needed<- (time_needed + (seg.perimeter/(speed)));
					
				}
			}
		}
	}
	
	//the agent follows the path it computed but with the real weights of the graph
	reflex move when: path_to_follow!=nil and (state contains 'go_'){
	
		if starting{
			departure_time<- current_hour;
			starting<-false;
		}
		do follow path:path_to_follow speed: 30.0#m/#s move_weights: graph_weights;
		if the_target = location {
			arrival_time<- current_hour;
			time_to_cover<- arrival_time-departure_time;
			dist_covered_alone<-dist_covered_alone + dist;
			dist<-0.0;
			the_target <- nil ;
			time_needed<-0.0;
			path_to_follow<-nil;
			starting<-true;
		}
	}
	
	aspect base {
		draw circle(25) color: color;
	}
}

