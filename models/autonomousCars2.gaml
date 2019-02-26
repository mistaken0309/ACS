/***
* Name: autonomousCars
* Author: Annalisa
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model autonomousCars 
global{
	
	file shape_file_roads  <- file("../includes/roads.shp") ;
	file shape_file_nodes  <- file("../includes/nodes.shp");
	file shape_file_buildings  <- file("../includes/buildings.shp");
	geometry shape <- envelope(shape_file_roads);
	
	graph the_graph; 
	graph road_network; 
	map<road, float> general_speed_map;

	// tolerable distance to current destination in order to give a lift and drop somebody
	// To avoid situations in which the car gets stuck a few meters away from the destination
//	float tolerated_distance <- 0.0000000000005;
	
	float cars_tot_distance_covered<-0.0#m;
	float people_tot_distance_covered<-0.0#m;
	float sys_tot_distance_covered<-0.0#m;
	float cars_tot_distance_covered_up<-0.0#m  update:     cars_tot_distance_covered#m;
	float people_tot_distance_covered_up<-0.0#m update: sum(people collect each.dist_covered_alone)#m;
	float sys_tot_distance_covered_up<-0.0#m update: people_tot_distance_covered_up#m+cars_tot_distance_covered_up#m;
	
	
	float step <- 1 #mn;		
	//Stock the number of times agents reached their goal (their house or work place)
	//int nbGoalsAchived <- 0;
	
	// factor added to slow down the simulation
	int factor<-2;
	
	//represent the day time for the agent to inform them to go work or home
	int current_hour update: ((time / #h)) mod (24*factor);
	int h update: (time/#hour/factor) mod 24; 
	int g update: int(time/#hour/factor/24);
	

	//Number of people created
	int n_people <- 500;
			
	//Variables to manage the minimal and maximal time to start working/go home
	int min_work_start <- (4*factor);
	int max_work_start <- (6*factor);
	int min_work_end <- (17*factor);
	int max_work_end <- (18*factor);
	
	//Manage the speed allowed in the model for the people agents
	float min_speed <- 20.0  #km / #h;
	float max_speed <- 30.0 #km / #h; 	
	
	init {
		create n from: shape_file_nodes with:[is_traffic_signal::(string(read("type")) = "traffic_signals")];
		ask n where each.is_traffic_signal {
			stop << flip(0.5) ? roads_in : [] ;
		}
		create road from: shape_file_roads with:[lanes::int(read("lanes")), maxspeed::float(read("maxspeed")) °km/°h, oneway::string(read("oneway"))] {
			geom_display <- (shape + (2.5 * lanes));

			switch oneway {
				match "no" {
					create road {
						lanes <- myself.lanes;
						shape <- polyline(reverse(myself.shape.points));
						maxspeed <- myself.maxspeed;
						geom_display  <- myself.geom_display;
						linked_road <- myself;
						myself.linked_road <- self;
						oneway<-'no';
					}
				}
				match "-1" {
					shape <- polyline(reverse(shape.points));
				}
			}
		}

		road_network <-  (as_driving_graph(road, n));

		ask n where (empty(each.roads_out) and empty(each.roads_in)){
			write string(self.name) + " died";
			do die;
		}
		ask n where (empty(each.roads_in)){
			if !empty(roads_out){
				write string(self.name) + " roads_in empty";
				road to_duplicate<- road (one_of (roads_out));
				n s_node<- n(to_duplicate.source_node);
				n t_node<- n(to_duplicate.target_node);
				if !empty(s_node.roads_in) or !empty(t_node.roads_out){
					if to_duplicate.linked_road=nil{
						create road {
							lanes <- to_duplicate.lanes;
							shape <- polyline(reverse(to_duplicate.shape.points));
							maxspeed <- to_duplicate.maxspeed;
							geom_display  <- to_duplicate.geom_display;
							linked_road <- to_duplicate;
							to_duplicate.linked_road<-self;
							created_on<-true;
						}
					}
					add to_duplicate.linked_road to: roads_in;
					roads_in<-remove_duplicates(roads_in);
				}
			} else{
				write string(self.name) + " died";
				do die;
			}
		}
		ask n where (empty(each.roads_out)){
			if !empty(roads_in){
				write string(self.name) + " roads_out empty";
				road to_duplicate<- road (one_of (roads_in));
				n s_node<- n(to_duplicate.source_node);
				n t_node<- n(to_duplicate.target_node);
				if !empty(s_node.roads_in) or !empty(t_node.roads_out){
					if to_duplicate.linked_road=nil{
						create road {
							lanes <- to_duplicate.lanes;
							shape <- polyline(reverse(to_duplicate.shape.points));
							maxspeed <- to_duplicate.maxspeed;
							geom_display  <- to_duplicate.geom_display;
							linked_road <- to_duplicate;
							to_duplicate.linked_road<-self;
							created_on<-true;
						}
					}
					add to_duplicate.linked_road to: roads_out;
					roads_out<-remove_duplicates(roads_out);
				}
				
			} else{
				write string(self.name) + " died";
				do die;
			}
		}
		
		create building from: shape_file_buildings with:[type::string(read("type")), group::string(read("group"))]; 
		ask building{
			if group='residential'{
				color <- #lightblue;
			} else{
				color<-#blue;
			}
		}
		write "created buildings";
		list<building> living_buildings<- building where (each.group='residential');
		list<building> work_buildings <-building where (each.group='industrial');
		
		create people number: n_people { 
			speed <- min_speed + rnd (max_speed - min_speed) ;
			start_work <- rnd (min_work_start,max_work_start, (factor/2));
			end_work <- rnd (min_work_end,max_work_end, (factor/2));
			living_place <- one_of(living_buildings) ;
			working_place <- one_of(work_buildings) ;
			location <- living_place.location; 
		}
		write "created people";
		
		the_graph <- as_edge_graph(road);
		int length_graph <- length(the_graph);
		
		general_speed_map <- road as_map (each::(each.shape.perimeter / (each.speed_coeff * each.maxspeed)));
		write general_speed_map;
		road_network <-  (as_driving_graph(road, n))  with_weights general_speed_map;
		write "Graph of length: "+ length_graph+ " created";
	}
	
	reflex update_road_speed  when: (current_hour<min_work_start-3 and current_hour>max_work_start+3) 
	or (current_hour>=min_work_end and current_hour>max_work_end+3) {
		general_speed_map <- road as_map (each::(each.shape.perimeter / (each.speed_coeff * each.maxspeed)));
		road_network <- road_network with_weights general_speed_map;
		the_graph<- the_graph with_weights general_speed_map;
	}
	
	reflex create_groups  when:( ( ( current_hour >(min_work_start-(factor/2)) ) and ( current_hour < (max_work_start-(factor/4)) ) ) or ( (current_hour >min_work_end) and (current_hour <(max_work_end +(factor/2))) ) ){
		string actual_state;
		string release_state;
		if  ( current_hour >(min_work_start-(factor/2)) ) and ( current_hour < (max_work_start-(factor/4)) ){
			actual_state <- 'search_lift_work';
			release_state <- 'working' ; 
		}
		if (current_hour >min_work_end) and (current_hour <(max_work_end +(factor/2))) {
			actual_state <- 'search_lift_home';
			release_state <- 'resting';
		}
		list<list<people>> people_in_range <- (people where ((each.state=actual_state )and each.the_target!=nil) simple_clustering_by_distance 1 )  where (/*( (length (each)) <=5) and*/ ( (length (each)) >0) ) ;
		if(people_in_range!=[]){
			write "___________________GROUPS @ " + h+" ("+ current_hour+") DAY "+(g+1)+"___________________";
			loop bigger over: people_in_range where (length(each)>5){
//				write "group longer than 5 " +bigger;
				loop while: length(bigger)>5{
					list<people> five<-copy_between(bigger,0,5);
//					write five;
					remove all:five from: bigger;
//					write bigger;
					add five to: people_in_range;
					five<-nil;
				}
			}
			loop one_group over: people_in_range{
				list<string> names<-one_group collect each.name; 
//				write string(names) + " - "+string(one_group); 
				loop p over: one_group{
					p.state<- 'wait_for_lift';
				}
				list<point>t_2 <- one_group collect each.location;
				t_2 <- remove_duplicates(t_2);
								
				create cars{
					location <- one_of(n where empty(each.stop)).location;
					give_lift_to<-one_group;
					p_targets <- t_2; // initialized with passengers current locations;
					next_people_state<-release_state;
					speed <- 10.0 #km /#h ;
					vehicle_length <- 4.0 #m;
					right_side_driving <- true;
					proba_lane_change_up <- 0.1 + (rnd(500) / 500);
					proba_lane_change_down <- 0.5+ (rnd(500) / 500);
					security_distance_coeff <- 2 * (1.5 - rnd(1000) / 1000);  
					proba_respect_priorities <- 1.0 - rnd(200/1000);
					proba_respect_stops <- [1.0 - rnd(2) / 1000];
					proba_block_node <- rnd(3) / 1000;
					proba_use_linked_road <- 0.0;
					max_acceleration <- 0.5 + rnd(500) / 1000;
					speed_coeff <- 1.2 - (rnd(400) / 1000);
					write string(self.name)+ " created for passengers: " + one_group; 
				}				
			}
		}
	}
}

species n skills: [skill_road_node] {
	bool is_traffic_signal;
	int time_to_change <- 2;
	int counter <- rnd (time_to_change) ;
	
	reflex dynamic when: is_traffic_signal {
		counter <- counter + 1;
		if (counter >= time_to_change) { 
			counter <- 0;
			stop[0] <- empty (stop[0]) ? roads_in : [] ;
		} 
	}
	
	aspect geom3D {
		if (is_traffic_signal) {	
			draw box(1,1,10) color:rgb("black");
			draw sphere(5) at: {location.x,location.y,12} color: empty (stop[0]) ? #green : #red;
		}
		else {
			draw square(5) color:#red;
		}
	}
}
species road skills: [skill_road] { 
	string oneway;
	geometry geom_display;
	bool created_on<-false;
	float capacity <- 1 + shape.perimeter/30;
	//Number of people on the road
	int nb_agents <- 0 update: length(people at_distance 1) + length(cars at_distance 1);
	//Speed coefficient computed using the number of people on the road and the capicity of the road
	float speed_coeff <- 1.0 update:  exp(-nb_agents/capacity) min: 0.1;
	int ov_rgb<-100 update: int((speed_coeff-0.1)*100.0);
	int ov_rgbR<-100 update: int(1.0-speed_coeff)*255 min: 100;
	rgb color<-rgb(127,127,127) update: rgb(ov_rgbR, ov_rgb, ov_rgb);
	aspect geom {    
//		draw geom_display border:  #gray  color: #gray ;
		draw geom_display color: color;
	}  
}
species building{
	string type;
	string group;
	rgb color<- rgb(200,200,200);
	aspect base{
		draw shape color: color border: #gray;
	}
}
species people skills:[moving] control: fsm {
	rgb color <- #lightblue ;
	building living_place <- nil ;
	building working_place <- nil ;
	int start_work ;
	int end_work ;
	point the_target <- nil ;
	float dist<-0.0 ;
	float dist_covered_alone ;
	bool late<-false;
	float time_needed<-0.0;
	float actual_time_in;

	state resting initial:true{
		enter{
			color <- #lightblue;
		}
		transition to: search_lift_work when: current_hour = start_work-(factor/2);
	}
	state search_lift_work{
		enter{
				the_target <- working_place.location;
				color<- #yellow;
				late<-false;
		}
		transition to: go_work when: current_hour = start_work -(int(factor/4));
	}
	state wait_for_lift{
		if current_hour>start_work and current_hour<end_work{
//			write string(self.name)+" getting late " + string(current_hour) + " " + string(start_work);
			late<-true;
		}else{
			late<-false;
		}
	}
	state go_work{
		enter{
				the_target<-working_place.location;
				if current_hour>start_work{
//					write string(self.name)+" getting late " + string(current_hour) + " " + string(start_work);
					late<-true;
				} else{
					late<-false;
				}
			}
		transition to: working when: current_hour >= start_work and self.location = working_place.location;
	}
	state working{
		enter{
			color <- #blue;
			actual_time_in<-time/3600;
			if late{
			write string(self.name) + " ATI "+actual_time_in + " SUPPOSED "+start_work ;}
		}
		transition to: search_lift_home when: current_hour = end_work;
	}
	state search_lift_home{
			enter{
					the_target <- living_place.location;
					color<- #yellow;	
					late<-false;			
			}
			transition to: go_home when: current_hour = end_work+(factor/2);
	}
	state go_home{
			enter{
				the_target<-living_place.location;
			}
			transition to: resting when: self.location = living_place.location;		
	}
	 
	reflex move when: the_target!=nil and (state="go_home" or state="go_work"){
		path path_followed <- self goto [target::the_target, on::the_graph, return_path:: true, move_weights::general_speed_map];
		list<geometry> segments <- path_followed.segments;
//		list<road> segments <- path_followed.edges;
		loop seg over:segments{
			dist <- (dist + seg.perimeter );
			time_needed<- (time_needed + (seg.perimeter/(speed)));
			
		}
//		write self.name+ " h"+current_hour/2+ " time needed "+time_needed + " for " + dist + "meters";
		if the_target = location {
			dist_covered_alone<-dist_covered_alone + dist;
			dist<-0.0;
			the_target <- nil ;
			time_needed<-0.0;
		}
	}
	
	aspect base {
//		draw triangle(50) color: color rotate: 90 + heading;
		draw circle(15) color: color;
	}
}


species cars skills:[advanced_driving] control:fsm{
	list<people> give_lift_to<-nil;
	string next_people_state<-nil;
	list<point> p_targets;
	list<point> been_to;
	point the_target;
	n the_node;
	list<n> to_avoid;
	float dist<-0.0 ;
	float dist_covered_cars<-0.0 ;
	float time_needed<-0.0;
	float starting_time;
	float arrived_time;
	geometry shape <- rectangle(50,100);
	int counter_stucked<-0;
	bool stuck<-false;
	float stuck_beg<-0.0;
	float stuck_end<-0.0;
	float stuck_remove<-0.0;
	
	state moving initial:true{
	} 
	state stop{
	} 
	
	reflex chose_new_target when: the_target=nil and state='moving'{
		to_avoid<-nil;
		if !empty(give_lift_to){
			list<point> t <- give_lift_to collect each.location;
			add all:t to:p_targets;
		}
		if !empty(passenger){
			list<point> destinations <- passenger collect each.the_target.location;
			add all:destinations to:p_targets;
		}
		p_targets<-remove_duplicates(p_targets);
		
		the_target<- first( list(p_targets) sort_by (each distance_to (location)));
		the_node<- n closest_to the_target;
//		write "\n"+string(self.name)+" h"+current_hour+ " chose new target: " + the_target + " from " +p_targets+ " and the closest node is " + the_node +" current road is "+ current_road;
	}

	reflex time_to_go when: final_target = nil and the_target!=nil and state='moving'{ 
		current_path <- compute_path(graph: road_network, target: the_node);
//		write current_path;
		if current_path!=nil{
			list<road> roads_in_path <- current_path.edges;
			float according_map<-0.0;
			loop r over:roads_in_path{
				dist <- dist+r.shape.perimeter;
				time_needed<- (time_needed + (r.shape.perimeter/(r.maxspeed *r.speed_coeff)));
				according_map<- according_map + general_speed_map[r];
			}
			write string(self.name)+" h"+current_hour +" Estimated time to cover " + dist+" is " +time_needed + " seconds ("+according_map+")";
			/*+ " current_path: "+current_path +"\n towards node "+the_node+" corresponding to current target: "+the_target+" from pass_targets: "+
			pass_targets+" current road is "+ current_road */
			starting_time<- time;
//			write string(self.name)+ " " + string(time) + " " + string(cycle) + " " + string(step);
		} else if current_path=nil and length(to_avoid)<11{
			add the_node to: to_avoid;
			the_node<- (n-to_avoid) closest_to the_target; 
		} else if current_path=nil and length(to_avoid)>10{
			if !empty(passenger) and !empty(give_lift_to){
				write string(self.name)+" h"+current_hour+ " impossible to do it. Dropping passengers "+ passenger + " and notifying people waiting " +give_lift_to;
			} else if !empty(passenger) and empty(give_lift_to){
				write string(self.name)+" h"+current_hour+ " impossible to do it. Dropping passengers "+ passenger;
			}  else if empty(passenger) and !empty(give_lift_to){
				write string(self.name)+" h"+current_hour+ " impossible to do it. Notifying people waiting " +give_lift_to;
			}  else if empty(passenger) and empty(give_lift_to){
				write string(self.name)+" h"+current_hour+ " impossible to do it.";
			} 
			string substitute_state;
			point t ;
			if next_people_state='working'{
				substitute_state<-'go_work';
			}
			if next_people_state='resting'{
				substitute_state<-'go_home';
			}
			if !empty(passenger){
				loop p over: (passenger){	
					release p in:world as:people{
						name<-name;
						location<-myself.location;
						state<- substitute_state;
						living_place <- living_place ;
						working_place <- working_place;
						start_work <- start_work;
						end_work <- end_work;
						dist_covered_alone<-dist_covered_alone;
						dist<-0.0;
						late<-late;
						write string(self.name) + " released at h" + string(time/3600/factor)+ " with state "+ self.state;
					}
					
				}
			}
			if !empty(give_lift_to){
				loop p over:give_lift_to{
					p.state<-substitute_state;
//					write string(p.name) + " notified at h" + string(time/3600/factor)+ " with state "+ p.state;
				}
			}
			do to_die;
		}	
	}
	reflex move when: final_target != nil and state='moving'{
		do drive;
		if real_speed < 5°km/°h {
			counter_stucked<- counter_stucked + 1;
//			write string(self.name) +" h" + time/factor/3600 + " got stuck " + counter_stucked;
			stuck<-true;
			stuck_beg<-time;
			
		}
		if stuck and real_speed>0°km/°h {
			stuck<-false;
			stuck_end<-time;
			stuck_remove<-stuck_remove+ ((stuck_end-stuck_beg)/2);
//			write string(self.name) +" h" + time/factor/3600 + " got unstuck " + counter_stucked + " "+ stuck_remove;
			stuck_beg<-0.0;
			stuck_end<-0.0;
		}
//		write string(self.name)+ " still have "+ self.location distance_to the_target + " to "+ the_target +" and " +self.location distance_to the_node + " to node "+the_node+" current road is "+ current_road;
//		write string(self.name)+ " "+ string(self.location distance_to the_node);
		if 	(self.location distance_to the_node)=0{
			counter_stucked<-0.0;
			arrived_time<-time;
			final_target<-nil;
			write string(self.name)+ " h"+current_hour+ " got to destination in "+(arrived_time/2#s-starting_time/2#s) + " - "+ stuck_remove + " = " + string((arrived_time/2#s-starting_time/2#s)-stuck_remove); //+ " "+ starting_time#s + " "+ arrived_time#s  *///+". Current road is "+ current_road;
			dist_covered_cars<-dist_covered_cars + dist;
//			write string(self.name)+' dist covered till now '+ dist_covered_cars; 
			cars_tot_distance_covered<-cars_tot_distance_covered + dist;
			dist<-0.0;
			time_needed<-0.0;
			stuck_remove<-0.0;
			state<-'stop';
		}
	}

	reflex get_on_board when:!empty(give_lift_to) and state='stop'{
		list<people>toremove;
		list<string>names;
		loop p over: give_lift_to{
//			write string(self.name)+ " the_target " +the_target + " p location "+p.location +" are equal " + string(p.location=the_target);
			if p.location=the_target{
				if p.the_target!=nil{
					add p to: toremove;
					add p.name to: names;
					point t<-p.the_target;
					write string(self.name)+ " capturing "+ p.name;
					capture p as:passenger{
						name<-name;
						state<-'getting_a_lift';
						color <- nil ;
						the_target<-t;
						living_place <- living_place ;
						working_place <- working_place;
						start_work <-start_work;
						end_work <-end_work;
						dist_covered_alone<-dist_covered_alone;
						dist<-0.0;
						late<-late;
					}
				}
			}
		}
		remove all:toremove from:give_lift_to;
		if !empty(toremove){
			//write string(self.name) +' ('+current_hour+') captured '+(names);//+' @ '+ location +' with: '+p_targets+ ' - ' +state;
		}
		remove the_target from: p_targets;
		add the_target to: been_to;
		the_target<- nil;
		state<-'moving';
		
	}
	reflex drop_people when:!empty(passenger) and state='stop'{
		list<string>names;
		list<people>dropped;
		string substitute_state;
		loop p over: (passenger){
			if p.the_target = the_target{
				add p to: dropped;
				add p.name to: names;
				point t<-p.the_target;
				if p.the_target!=location{
					if next_people_state='working'{
						substitute_state<-'go_work';
					}
					if next_people_state='resting'{
						substitute_state<-'go_home';
					}
				}else{
					substitute_state<-next_people_state;
				}
				release p in:world as:people{
					name<-name;
					location<-myself.location;
					state<- substitute_state;
					the_target<-t;
					living_place <- living_place ;
					working_place <- working_place;
					start_work <- start_work;
					end_work <- end_work;
					dist_covered_alone<-dist_covered_alone;
					dist<-0.0;
					late<-late;
				}
			}
		}
		if !empty(dropped){			
			write string(self.name) +' ('+current_hour+') dropped '+(names); //+' @ '+ location +' with: '+p_targets+ ' - ' +state;
		}
		if empty(passenger){
//			write string(self.name) +' ('+current_hour+') DIED';
			do to_die;
		}
		remove the_target from: p_targets;
		add the_target to: been_to;
		the_target<- nil;
		state<-'moving';
	}
	action to_die{
		ask road where !empty(each.all_agents) {
			if all_agents contains myself{
//				write self.name + " " + self.all_agents+ " " + myself;
//				write "removing " + myself + " from " + self.name + " all_agents";
				remove myself from: all_agents;
				loop s over: agents_on{
//					write s;
					loop l over:s{
//						write l;
						if list(l) contains myself{
							remove myself from: list(l);
//							write "removing " + myself + " from " + self.name + " agents_on";
						}
					
					}
				}				
			}
		}
		current_road<-nil;
		location<-nil;
		write string(self.name) +' ('+current_hour+') DIED';
		do die;
	}
	
	species passenger parent: people{
		rgb color;
		building living_place;
		building working_place;
		int start_work ;
		int end_work ;
		point the_target;
		float dist_covered_alone;
		float dist;
		
		state getting_a_lift{
		}
		aspect default{
		}
	}
	aspect base{
		draw shape color: #red border: #darkred;	
	}
	aspect realistic{
		draw image_file("../includes/car-png-top-white-top-car-png-image-34867-587.png") size:{24,12} rotate: heading+180;
//		draw image_file("../includes/car-png-top-white-top-car-png-image-34867-587.png") size:{8.6#m,3.6#m} rotate: 90 + heading;
//		draw obj_file("../includes/pod_glider_30.obj") color:#steelblue rotate: 90 + heading;
//		draw obj_file("../includes/pod_glider_30.obj") color:rgb(0,51,153) rotate: 90 + heading;
	}
	
}


experiment liftToAndFromWork type: gui {
	float minimum_cycle_duration <- 0.001;
	output {
		monitor "Current hour" value: current_hour/factor;
		monitor "Current hour (t/f/3600)" value: (time/factor/3600) ;
		monitor "Tot distance covered" value: sys_tot_distance_covered_up #m;
		monitor "Tot distance cars" value: cars_tot_distance_covered_up #m;
		monitor "Tot distance people" value: people_tot_distance_covered_up #m;
//		display my_display {
//			chart "my_chart" type:pie {
//				data "car_kilometers" value:(cars_tot_distance_covered_up/1000) color:#red;
//				data "people_kilometers" value:(people_tot_distance_covered_up/1000 ) color:#blue;
//			}
//		}
		display map type: opengl{
			graphics "world" {
				draw world.shape.contour;
			}
			species road aspect: geom transparency: 0.2 refresh:true;
			species n aspect: geom3D transparency: 0.5 refresh:true;
			species building aspect: base transparency: 0.2 refresh:false;
			species people aspect: base transparency: 0.1 refresh: true ;
			species cars aspect: realistic transparency: 0.1 refresh: true ;
		}
//		display series refresh:every(5 #mn) {
//			chart "Total distance" type: series {
//				data "system" value: sys_tot_distance_covered_up color: #blue;
//				data "cars" value: cars_tot_distance_covered_up color: #green;
//				data "people" value: people_tot_distance_covered_up color: #red;
//			}
//		}
//		display histogram refresh:every(10 #mn) {
//			chart "Total distance" type: histogram 
//			x_serie_labels:("cycle"+cycle){
//				data "system" value: sys_tot_distance_covered_up color: #blue;
//				data "cars" value: cars_tot_distance_covered_up color: #green;
//				data "people" value: people_tot_distance_covered_up color: #red;
//			}
//		}
//		display "data_pie_chart" type: java2D
//		{
//			chart "Nice Ring Pie Chart" type: pie style: ring background: # white color: # lightgreen axes: # yellow title_font: 'Serif' title_font_size: 32.0 title_font_style: 'italic'
//			tick_font: 'Monospaced' tick_font_size: 14 tick_font_style: 'bold' label_font: 'Arial' label_font_size: 32 label_font_style: 'bold' x_label: 'Nice Xlabel' y_label:
//			'Nice Ylabel'
//			{
//				data "people late" value: people count (each.late=true) color: #pink;
//				data "people on time" value: people count(each.late=false) color: #lightgreen;
//			}
//
//		}
	}
}