/**
* Name: cenAV
* Author: Annalisa Congiu
* Description: species representing AV in the centralised simulation
* Tags: 
*/
model cenAV
import "./../models/common_agents.gaml"

/* Species representing Autonomous Vehicles in the centralised simulation. */

species cenAV parent:cars skills: [advanced_driving] control:fsm{ 
	intersection the_target;
	float time_needed;
	float dist <-0.0;
	float dist_covered_cars<-0.0;
	bool gave_lift <- false;
	int n_travels <- 0;
	
	map<intersection, list<people>> give_lift_to;
	map<intersection, list<string>> people_destinations;
	map<list<intersection>, list<float>> cost_legs;
	list<people> all_passengers;
	map<string, list<float>> cost_passengers<-nil; // list of costs per each passenger
	
	int max_passengers<-5;
	int passengers_waiting<-0;
	int on_board<- 0 update: length(passengers) min:0;
	int time_to_go<-rnd(12,24);
	int counter<-0;
	
	map<list<string>, float> waiting_time<-nil;
	float mean_of_costs<-0.0;
	float max_of_costs<-0.0;
	float min_of_costs<-0.0;
	
	float arrived_time<-0.0;
	float starting_time<-0.0;	
	
	state wander initial: true{}
	state still_place{}
	state moving{} 
	state stop{}
	state computing{}
		
	reflex time_to_go when: final_target = nil{
		if state='wander'{
			the_target <- one_of(intersection where not each.is_traffic_signal);
			current_path <- compute_path(graph: road_network, target: the_target );
			if (current_path = nil) {
				final_target <- nil;
			}else{starting_time<- time;}
		}
	}
	
	action reset_vars_for_wander{
		state<- 'wander';
		the_target<-nil;
		
		passengers_waiting<-0;
		on_board<-0;
		all_passengers<-nil;
		
		
		give_lift_to<-nil;
		people_destinations<-nil;
		cost_legs<-nil;
		cost_passengers<-nil;
		waiting_time<-nil;
		
		mean_of_costs<-0.0;
		max_of_costs<-0.0;
		min_of_costs<-0.0;
		
		counter<-0;
	}	
	
	action chose_new_target{
//		write self.name + " lift: " + give_lift_to.keys + " check " + (!empty(give_lift_to.keys))+ " " + people_destinations.keys;
		if !empty(give_lift_to.keys){
			the_target<- first(give_lift_to.keys);
		}else{
			the_target<- first(people_destinations.keys);
		}
		write self.name + " chosen "+ the_target+" as new target";
	}
	
	reflex get_path_moving when: final_target = nil and state='moving'{
		loop while: the_target=nil{
			do chose_new_target;
//			write '\n'+self.name+ ' chosed new_target: '+ the_target;
		}
		current_path <- compute_path(graph: road_network, target: the_target );
		if current_path!=nil{
			list<road> roads_in_path <- list<road>(current_path.edges);
			loop r over:roads_in_path{
				dist <- dist+r.shape.perimeter;
				time_needed<- (time_needed + (r.shape.perimeter/r.maxspeed));
			}
//			write string(self.name)+" h"+current_hour + " from "+ intersection overlapping self.location +" to "+ the_target+" Estimated time to cover " + dist+" is " +time_needed + " seconds";
			starting_time<- time;
		}
	}

	reflex move when: current_path != nil and final_target != nil{
		if(state='wander' or state='moving'){
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
			
			if (self.location = the_target.location){
				arrived_time<-time;
				final_target<-nil;
				dist_covered_cars<-dist_covered_cars + dist;
				dist<-0.0;
				time_needed<-0.0;
				if state='moving'{state<-'stop';}
			}
		}		
	}
	reflex waiting_other_passengers when: state='still_place'  {
		counter <- counter + 1;
		if (counter >= time_to_go or passengers_waiting=max_passengers) { 
			counter <- 0;
			the_target<-nil;
			final_target<-nil;
			current_path<-nil;
			state<-'computing';
			write self.name + " h"+current_hour+" setting off. (Passengers waiting: "+passengers_waiting + ". Time_left: "+(time_to_go-counter)*60;
		} 
	}
	reflex create_path_costs when:!empty(give_lift_to) and state='computing' {
		stats_path_time<- machine_time;
		write self.name + " give_lift_to: "+ give_lift_to ;
		list<intersection> sorted_origins <- give_lift_to.keys;
		list<intersection> sorted_destinations <- (people_destinations.keys) sort_by (road_network path_between(last(give_lift_to.keys), each));
		
		map<intersection, list<string>> tmp <- people_destinations;
		people_destinations<-nil;
		
		
		int people_on_leg <- 0;
		float time_to_reach<- 0.0;
		path leg<-nil;
		float cost<-0.0;
		list<intersection> new_key;
		float time_leg<- 0.0;
		
		write self.name + " h"+current_hour+" COMPUTING COSTS";
		write self.name + " all passengers: " + all_passengers;
		write self.name + " origins: " + sorted_origins + " destinations: " +sorted_destinations;
		
		int i<-0;
		write self.name + " computing costs for leg <"+ location + ", "+ sorted_origins[0] +">"; 
		leg <- road_network path_between(location, sorted_origins[0]);
		loop e over: list<road>(leg.edges){
			time_to_reach <- time_to_reach + (e.shape.perimeter/e.maxspeed);
			time_leg<- time_leg + (e.shape.perimeter/e.maxspeed);
		}
//		write self.name + " time to wait for "+ give_lift_to[sorted_origins[0]]+ " is "+ time_to_reach;

		add (give_lift_to[sorted_origins[0]] collect each.name)::time_to_reach to: waiting_time;
		
		loop times: length(give_lift_to.keys)-1{
			path leg<-nil;
			float cost<-0.0;
			list<intersection> new_key;
			float time_leg<- 0.0;
			
			people_on_leg<- people_on_leg + length(give_lift_to[sorted_origins[i]]);
			write self.name + " computing costs for leg <"+ sorted_origins[i] + ", "+ sorted_origins[i+1] +"> "+ people_on_leg + " passengers on";
			leg <- road_network path_between(sorted_origins[i], sorted_origins[i+1]);
			loop e over: list<road>(leg.edges){
				time_to_reach <- time_to_reach + (e.shape.perimeter/e.maxspeed);
				time_leg<- time_leg + (e.shape.perimeter/e.maxspeed);
			}
//			write self.name + " time to wait for "+ give_lift_to[sorted_origins[i+1]]+ " is "+ time_to_reach;
			add (give_lift_to[sorted_origins[i+1]] collect each.name)::time_to_reach to: waiting_time;
			cost <- (leg.shape.perimeter/1000)*cost_km;
			new_key <- [sorted_origins[i], sorted_origins[i+1]];
			add new_key::[cost, people_on_leg, time_leg] to:cost_legs;	
			
			i<- i+1;
		}
		
		i<-0;
		time_leg<-0.0;
		cost<-0.0;
		
		
		if last(sorted_origins)!=sorted_destinations[0]{
			people_on_leg<- people_on_leg + length(give_lift_to[last(sorted_origins)]);
			write self.name + " computing costs for leg <"+ last(sorted_origins) + ", "+ sorted_destinations[0] +"> with "+ people_on_leg + " passengers on"; 
			leg <- road_network path_between(last(sorted_origins), sorted_destinations[0]);
			loop e over: list<road>(leg.edges){
				time_leg<- time_leg + (e.shape.perimeter/e.maxspeed);
			}
			cost <- (leg.shape.perimeter/1000)*cost_km;
			new_key <- [last(sorted_origins), sorted_destinations[0]];
			add new_key::[cost, people_on_leg, time_leg] to:cost_legs;
			add sorted_destinations[0]::tmp[sorted_destinations[0]] to:people_destinations;
		} else{
			people_on_leg<- people_on_leg + length(give_lift_to[last(sorted_origins)])-length(tmp[sorted_destinations[i]]);
			i<-1;
			write self.name + " computing costs for leg <"+ last(sorted_origins) + ", "+ sorted_destinations[i] +"> with "+ people_on_leg + " passengers on";
			leg <- road_network path_between(last(sorted_origins), sorted_destinations[i]);
			loop e over: list<road>(leg.edges){
				time_leg<- time_leg + (e.shape.perimeter/e.maxspeed);
			}
			cost <- (leg.shape.perimeter/1000)*cost_km;
			new_key <- [last(sorted_origins), sorted_destinations[i]];
			add new_key::[cost, people_on_leg, time_leg] to:cost_legs;
			add sorted_destinations[0]::tmp[sorted_destinations[i]] to:people_destinations;
		}
		loop times: ((last(sorted_origins)!=sorted_destinations[0]) ? (length(sorted_destinations)-1) : (length(sorted_destinations)-2)){
			time_leg<-0.0;
			cost<-0.0;
			
			people_on_leg<- people_on_leg - length(tmp[sorted_destinations[i]]);
			write self.name + " computing costs for leg <"+ sorted_destinations[i] + ", "+ sorted_destinations[i+1] +"> with "+ people_on_leg + " passengers on";
			leg <- road_network path_between(sorted_destinations[i], sorted_destinations[i+1]);
			loop e over: list<road>(leg.edges){
				time_leg<- time_leg + (e.shape.perimeter/e.maxspeed);
			}
//			write self.name + "from: "+ sorted_destinations[i] + " to: "+ sorted_destinations[i+1] + " "+ leg;
			
			cost <- (leg.shape.perimeter/1000)*cost_km;
			new_key <- [sorted_destinations[i], sorted_destinations[i+1]];
			add new_key::[cost, people_on_leg, time_leg] to:cost_legs;
			add sorted_destinations[i+1]::tmp[sorted_destinations[i+1]] to:people_destinations;
			i<- i+1;
			
		}
		
//		write self.name + " waiting times: " + waiting_time + " cost_legs:"+cost_legs;
		
		int total_stops <- 1+ length(sorted_origins) + length(sorted_destinations);
		
		loop p over: all_passengers{
			
			intersection origin <- intersection closest_to (p.location);
			intersection dest <- intersection closest_to (p.the_target);
			write self.name + " computing costs for passenger "+ p.name + " from "+ origin+ "to "+ dest;
			
			cost<-0.0;
			time_leg<-0.0;
			
			list<string> key_waiting <- (waiting_time.keys where (each contains p.name))[0];
			
			write self.name + " waiting_time["+key_waiting+"]="+ waiting_time[key_waiting];
			
			time_to_reach <- waiting_time[key_waiting];
			
			bool next_too<-false;
			
			loop key over: cost_legs.keys{
				if key[0]=origin{
					next_too<-true;
					cost <-  cost + (cost_legs[key][0]/cost_legs[key][1]);
					time_leg <- time_leg + cost_legs[key][2];
				}
				if next_too{
					cost <- cost + (cost_legs[key][0]/cost_legs[key][1]);
					time_leg <- time_leg + cost_legs[key][2];
					if key[1]=dest{break;}
				}
			}
			add (p.name)::[cost, time_to_reach, time_leg] to: cost_passengers;
		}	
		write self.name + " waiting times: " + waiting_time + " cost_legs:"+cost_legs + " cost_passengers: "+cost_passengers;
		stats_path_time<- machine_time - stats_path_time;
		n_travels <- n_travels+1;
		int total_passengers<- length(cost_passengers.keys);
		list<float> costs_p <- cost_passengers.values collect each[0];
		list<float> waiting_times <- cost_passengers.values collect each[1];
		gave_lift<-true;
			
		write ((cenAV index_of self<10)? "0": "") + string(cenAV index_of self) +", " +n_travels +", " +stats_path_time+", " +total_stops+", " +total_passengers+", " +waiting_times+ ", "+ costs_p; 
		state<-'moving';
	}
	reflex capture_people when: !empty(give_lift_to) and state='stop'{
		write self.name + " the_target: "+ the_target + " | "+ give_lift_to[the_target]; 
		list<string>names <- give_lift_to[the_target] collect each.name;
		list<people> to_capture <- give_lift_to[the_target]; 
		remove key:the_target from: give_lift_to;
		write self.name + " removed the key " + the_target + " from " + give_lift_to;
		ask to_capture{
			got_lift<-true;
			cost_proposed <- myself.cost_passengers[name][0];
			time_trip <- myself.cost_passengers[name][2];
			total_time_needed <- waiting_time + time_trip;
		}
		capture to_capture as: passengers{} 
		write self.name + " h"+current_hour+" at: " + the_target + " has captured: " + names;
		
		the_target<-nil;
		state<-'moving';
		write self.name + " h"+current_hour+" changed state back to moving";
	}
	reflex drop_people when:!empty(passengers) and state='stop'{
		list<string>names;
		list<string> states_dropped;
		list<people>dropped;
		string substitute_state;
		
//		write self.name + " Passengers on before droppping someone: " + list(passengers);
//		write self.name + " People destinations: " + people_destinations + " current_target "+ the_target ;
		loop p over: (passengers){
			if people_destinations[the_target] contains p.name and self.location=the_target.location{
//				write self.name + " People destinations[target]: " + people_destinations[the_target] + " - p "+ p + "="+ p.name;
				add p to: dropped;
				add p.name to: names;
				point t<-p.the_target;
				if p.the_target!=location{
					if (p.next_state='working' or p.next_state='go_work') { //next_state
						substitute_state<-'go_work';
					}
					if (p.next_state='resting' or p.next_state='go_home') {
						substitute_state<-'go_home';
					}
				}else{
					substitute_state<-p.next_state;
				}
				add substitute_state to: states_dropped;
				ask p{
					state<- substitute_state;
					location<-myself.location;
					path_to_follow<-nil;
				}
				release p in:world as:people{}
			}
		}
		if !empty(dropped){
			write string(self.name) +' ('+current_hour+') dropped '+(names) + ' with states: '+ states_dropped; //+' @ '+ location +' with: '+p_targets+ ' - ' +state;
		}
		
		remove key:the_target from: people_destinations;
//		write self.name + ' removed the_target from int_targets and from people_destinations and then the_target is set to nil';
		write self.name + ' - '+ people_destinations;
		the_target<-nil;
		if empty(passengers){
			do reset_vars_for_wander;
			write self.name + " back to wandering";
		} else{state<-'moving';}
	}
	
	species passengers parent: people{
		aspect default{
		}
	}

	aspect base {
		draw image_file("../includes/car-png-top-white-top-car-png-image-34867-587.png") size:{5#m,2.5#m} rotate: heading+180;
		draw breakdown ? square(15) : triangle(30) color: color rotate:heading + 90;
	} 
	aspect base3D {
		point loc <- calcul_loc();
		draw box(vehicle_length, 1,1) at: loc rotate:  heading color: color;
		
		draw triangle(0.5) depth: 1.5 at: loc rotate:  heading + 90 color: color;
		
		if (breakdown) {
			draw circle(2) at: loc color: #red;
		}
	} 
	
} 


