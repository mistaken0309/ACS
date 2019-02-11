/***
* Name: autonomousCars
* Author: Annalisa
* Description: 
* Tags: Tag1, Tag2, TagN
***/

model autonomousCars1 
global{
	
	file shape_file_roads  <- file("../includes/roads.shp") ;
	file shape_file_nodes  <- file("../includes/nodes.shp");
	file shape_file_buildings  <- file("../includes/buildings.shp");
	geometry shape <- envelope(shape_file_roads);
	
	graph the_graph; 
	graph road_network; 
	map general_speed_map;

	// tolerable distance to current destination in order to give a lift and drop somebody
	// To avoid situations in which the car gets stuck a few meters away from the destination
	float tolerated_distance <- 0.0000000000005;
	
	float cars_tot_distance_covered<-0.0 ;
	float people_tot_distance_covered<-0.0;
	float sys_tot_distance_covered<-0.0 update: people_tot_distance_covered+cars_tot_distance_covered;
	
	
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
	int n_people <- 10;
			
	//Variables to manage the minimal and maximal time to start working/go home
	int min_work_start <- (8*factor);
	int max_work_start <- (9*factor);
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

		general_speed_map <- road as_map (each::(each.shape.perimeter / (each.maxspeed)));
		road_network <-  (as_driving_graph(road, n))  with_weights general_speed_map;

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
		
		road_network <-  (as_driving_graph(road, n))  with_weights general_speed_map;
		
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
		general_speed_map <- road as_map (each::(each.shape.perimeter / (each.maxspeed)));
		write "Graph of length: "+ length_graph+ " created";
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
		list<list<people>> people_in_range <- (people where ((each.state=actual_state )and each.the_target!=nil) simple_clustering_by_distance 1 )  where (( (length (each)) <=5) and ( (length (each)) >0) ) ;
		if(people_in_range!=[]){
			write "___________________GROUPS @ " + h+" ("+ current_hour+") DAY "+(g+1)+"___________________";
			loop one_group over: people_in_range{
				list<string> names<-one_group collect each.name; 
				write string(names) + " - "+string(one_group); 
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
					speed <- 30 #km /#h ;
					vehicle_length <- 3.0 #m;
					right_side_driving <- true;
					proba_lane_change_up <- 0.1 + (rnd(500) / 500);
					proba_lane_change_down <- 0.5+ (rnd(500) / 500);
					location <- one_of(n where empty(each.stop)).location;
					security_distance_coeff <- 2 * (1.5 - rnd(1000) / 1000);  
					proba_respect_priorities <- 1.0 - rnd(200/1000);
					proba_respect_stops <- [1.0 - rnd(2) / 1000];
					proba_block_node <- rnd(3) / 1000;
					proba_use_linked_road <- 0.0;
					max_acceleration <- 0.5 + rnd(500) / 1000;
					speed_coeff <- 1.2 - (rnd(400) / 1000);
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
	aspect geom {    
		draw geom_display border:  #gray  color: #gray ;
	}  
}
species building{
	string type;
	string group;
	rgb color<- rgb(200,200,200);
	aspect base{
		draw shape color: color border: #black;
	}
}
species people skills:[moving] control: fsm {
	rgb color <- #lightblue ;
	building living_place <- nil ;
	building working_place <- nil ;
	int start_work ;
	int end_work ;
	point the_target <- nil ;
	float dist<-0.0;
	float dist_covered_alone;

	state resting initial:true{
		enter{
			color <- #lightblue;
		}
		transition to: search_lift_work when: current_hour = start_work-(factor/2);
	}
	state search_lift_work{
		enter{
				the_target <- working_place.location;
				color<- #green;
		}
		transition to: go_alone when: current_hour = start_work -(int(factor/4));
	}
	state wait_for_lift{
		
	}
	state go_alone{
		enter{
				the_target<-working_place.location;
			}
		transition to: working when: current_hour >= start_work and self.location = working_place.location;
	}
	state working{
		enter{
			color <- #blue;
		}
		transition to: search_lift_home when: current_hour = end_work;
	}
	state search_lift_home{
			enter{
					the_target <- living_place.location;
					color<- #cyan;				
			}
			transition to: go_home when: current_hour = end_work+(factor/2);
	}
	state go_home{
			enter{
				the_target<-living_place.location;
			}
			transition to: resting when: self.location = living_place.location;		
	}
	 
	reflex move when: the_target!=nil and (state="go_home" or state="go_alone"){
		path path_followed <- self goto [target::the_target, on::the_graph, return_path:: true];
		list<geometry> segments <- path_followed.segments;
		loop seg over:segments{
			dist <- dist+seg.perimeter;		
		}
		if the_target = location {
			dist_covered_alone<-dist_covered_alone+dist;
//			sys_tot_distance_covered<-sys_tot_distance_covered+dist;
			people_tot_distance_covered<-people_tot_distance_covered+dist;
			dist<-0.0;
			the_target <- nil ;
		}
	}
	
	aspect base {
		draw triangle(50) color: color border: #black;
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
	float dist<-0.0;
	float total_distance_covered<-0.0;
	float time_needed<-0.0;
	float starting_time;
	float arrived_time;
	geometry shape <- rectangle(50,100);
	
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
		write "\n"+string(self.name)+" h"+current_hour+ " chose new target: " + the_target + " from " +p_targets+ " and the closest node is " + the_node +" current road is "+ current_road;
	}

	reflex time_to_go when: final_target = nil and the_target!=nil and state='moving'{ 
		current_path <- compute_path(graph: road_network, target: the_node);
//		write current_path;
		if current_path!=nil{
			list<road> roads_in_path <- current_path.edges;
			loop r over:roads_in_path{
				dist <- dist+r.shape.perimeter;
				time_needed<- time_needed+ (r.maxspeed/r.shape.perimeter);
			}
			write string(self.name)+" h"+current_hour+ " current_path: "+current_path +"\n towards node "+the_node+" corresponding to current target: "+the_target+/*" from pass_targets: "+
			pass_targets+*/" current road is "+ current_road +" Estimated time to cover all path of lenght " + dist + " meters is " +time_needed + " seconds";
			starting_time<- time;
			write string(self.name)+ " " + string(time) + " " + string(cycle) + " " + string(step);
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
			
			if !empty(passenger){
				loop p over: (passenger){
					string substitute_state;
					if next_people_state='working'{
						substitute_state<-'go_alone';
					}
					if next_people_state='resting'{
						substitute_state<-'go_home';
					}
					release p in:world as:people{
						name<-name;
						location<-myself.location;
						state<- 'go_alone';
						living_place <- living_place ;
						working_place <- working_place;
						start_work <- start_work;
						end_work <- end_work;
						dist_covered_alone<-dist_covered_alone;
						dist<-0.0;
					}
				}
			}
			if !empty(give_lift_to){
				loop p over:give_lift_to{
					state<-'go_alone';
				}
			}
			do to_die;
		}	
	}
	reflex move when: final_target != nil and state='moving'{
		do drive;
//		write string(self.name)+ " still have "+ self.location distance_to the_target + " to "+ the_target +" and " +self.location distance_to the_node + " to node "+the_node+" current road is "+ current_road;
//		write string(self.name)+ " "+ string(self.location distance_to the_node);
		if 	(self.location distance_to the_node)=0{
			write string(self.name)+ " " +  string(time) + " " + string(cycle) + " " + string(step);
			arrived_time<-time;
			final_target<-nil;
			write string(self.name)+ " h"+current_hour+ " got to destination in "+(arrived_time-starting_time)+". Current road is "+ current_road;
			total_distance_covered<-total_distance_covered+dist;
//			sys_tot_distance_covered<-sys_tot_distance_covered+dist;
			cars_tot_distance_covered<-cars_tot_distance_covered+dist;
			dist<-0.0;
			time_needed<-0.0;
			state<-'stop';
		}
	}

	reflex get_on_board when:!empty(give_lift_to) and state='stop'{
		write string(self.name)+ " ready to get someone on bord";
		list<people>toremove;
		list<string>names;
		loop p over: give_lift_to{
			write string(self.name)+ " the_target " +the_target + " p location "+p.location +" are equal " + string(p.location=the_target);
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
					}
				}
			}
		}
		remove all:toremove from:give_lift_to;
		if !empty(toremove){
			write string(self.name) +' ('+current_hour+') captured '+(names) +' @ '+ location +' with: '+p_targets+ ' - ' +state;
		}
		remove the_target from: p_targets;
		add the_target to: been_to;
		the_target<- nil;
		state<-'moving';
		
	}
	reflex drop_people when:!empty(passenger) and state='stop'{
		write string(self.name)+ " ready to drop someone";
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
						substitute_state<-'go_alone';
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
				}
			}
		}
		if !empty(dropped){			
			write string(self.name) +' ('+current_hour+') dropped '+(names) +' @ '+ location +' with: '+p_targets+ ' - ' +state;
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
		draw obj_file("../includes/pod_glider_30.obj") color:rgb(255, 30, 100) rotate: 90 + heading;
//		draw obj_file("../includes/pod_glider_30.obj") color:rgb(0,51,153) rotate: 90 + heading;
	}
	
}


experiment liftToAndFromWork type: gui {
	float minimum_cycle_duration <- 0.01;
	output {
		monitor "Current hour" value: current_hour/factor;
		monitor "Total distance covered by the system" value: sys_tot_distance_covered/1000;
		monitor "Total distance covered by cars" value: cars_tot_distance_covered/1000;
		monitor "Total distance covered by people" value: people_tot_distance_covered/1000;
//		display my_display {
//			chart "my_chart" type:pie {
//				data "car_kilometers" value:(cars_tot_distance_covered/1000) color:#red;
//				data "people_kilometers" value:(people_tot_distance_covered/1000 ) color:#blue;
//			}
//		}
		display map type: opengl  background:rgb(0,0,15){
			graphics "world" {
				draw world.shape.contour;
			}
			species road aspect: geom transparency: 0.3 refresh:true;
			species n aspect: geom3D transparency: 0.5 refresh:true;
			species building aspect: base transparency: 0.2 refresh:true;
			species people aspect: base transparency: 0.2 refresh: true ;
			species cars aspect: realistic transparency: 0.5 refresh: true ;
		}
//		display chart refresh:every(5 #mn) {
//			chart "Total distance" type: series {
//				data "system" value: sys_tot_distance_covered color: #blue;
////				data "cars" value: cars_tot_distance_covered color: #green;
////				data "people" value: people_tot_distance_covered color: #red;
//			}
//		}
//		display chart{
//			chart "Numbers" type: pie{
//				data "people" value:length(people) color: #lightblue;
//				data "roads" value:length(road) color: #gray;
//			}	
//		}

		
	}
}