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
	map general_speed_map;

	// tolerable distance to current destination in order to give a lift and drop somebody
	// To avoid situations in which the car gets stuck a few meters away from the destination
	float tolerated_distance <- 0.0000000000005;
	
	float step <- 10 #mn;		
	//Stock the number of times agents reached their goal (their house or work place)
	//int nbGoalsAchived <- 0;
	
	// factor added to slow down the simulation
	int factor<-2;
	
	//represent the day time for the agent to inform them to go work or home
	int current_hour update: ((time / #h)) mod (24*factor);
	int h update: (time/#hour/factor) mod 24; 
	int g update: int(time/#hour/factor/24);
	

	//Number of people created
	int n_people <- 50;
			
	//Variables to manage the minimal and maximal time to start working/go home
	int min_work_start <- (8*factor);
	int max_work_start <- (9*factor);
	int min_work_end <- (17*factor);
	int max_work_end <- (18*factor);
	
	//Manage the speed allowed in the model for the people agents
	float stop_speed <- 0.0   #km / #h;
	float min_speed <- 20.0  #km / #h;
	float max_speed <- 30.0 #km / #h; 	
	
	init {
		create n from: shape_file_nodes with:[is_traffic_signal::(string(read("type")) = "traffic_signals")];
		ask n where each.is_traffic_signal {
			stop << flip(0.5) ? roads_in : [] ;
		}
		write "created nodes";
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
					}
				}
				match "-1" {
					shape <- polyline(reverse(shape.points));
				}
			}
		}	
		write "created roads";
		
		create building from: shape_file_buildings with:[type::string(read("type")), group::string(read("group"))]; 
		ask building{
			if group='residential'{
				color <- #lightblue;
			} else{
				color<-#blue;
			}
		}
		
		write "created buildings";
		general_speed_map <- road as_map (each::(each.shape.perimeter / (each.maxspeed)));
		road_network <-  (as_driving_graph(road, n))  with_weights general_speed_map;
		
		list<building> living_buildings<- building where (each.group='residential');
		list<building> work_buildings <-building where (each.group='industrial');
		
		create people number: n_people { 
//			living_space <- 3.0;
//			tolerance <- 0.1;
//			lanes_attribute <- "nbLanes";
//			obstacle_species <- [species(self)]; 
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
		string release_state;
		if  ( current_hour >(min_work_start-(factor/2)) ) and ( current_hour < (max_work_start-(factor/4)) ){
			release_state <- 'working' ; 
		}
		if (current_hour >min_work_end) and (current_hour <(max_work_end +(factor/2))) {
			release_state <- 'resting';
		}
		list<list<people>> people_in_range <- (people where ((each.state='search_lift' )and each.the_target!=nil) simple_clustering_by_distance 1 )  where (( (length (each)) <=5) and ( (length (each)) >0) ) ;
		if(people_in_range!=[]){
			write "___________________GROUPS @ " + h+" ("+ current_hour+") DAY "+(g+1)+"___________________";
			loop one_group over: people_in_range{
				write one_group;
				loop p over: one_group{
					p.state<- 'wait_for_lift';
				}
							
				list<point>t_2 <- one_group collect each.location;
				t_2 <- remove_duplicates(t_2);
								
				create cars{
					location<-one_of(one_group).location;
					give_lift_to<-one_group;
					targets <- t_2; // initialized with passengers current locations;
					current_target <- nil;
					next_people_state<-release_state;
				}
			}
		}
	}
}

species n skills: [skill_road_node] {
	bool is_traffic_signal;
	int time_to_change <- 100;
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
	}
}

species road skills: [skill_road] { 
	string oneway;
	geometry geom_display;
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

	state resting initial:true{
		enter{
			color <- #lightblue;
//			write string(self.name) +"setting state to working "+ current_hour + " (" +h+")";
		}
		transition to: search_lift when: current_hour = start_work-(factor/2);
	}
	state working{
		enter{
			color <- #blue;
		}
		transition to: search_lift when: current_hour = end_work;
	}
	state search_lift{
		enter{
			if(current_hour = start_work-(factor/2)){
				the_target <- working_place.location;
				color<- #green;
//				write string(self.name) +" searching a lift for work @ "+ current_hour + " (" +h+")";
			}
			if current_hour = end_work{
				the_target <- living_place.location;
				color<- #cyan;
//				write string(self.name) +" searching a lift home @ "+ current_hour + " (" +h+")";
			}
			
		}
		transition to: go_alone when: (current_hour = start_work -(factor/4)) or  (current_hour  = end_work+(factor/2));
	}
	state wait_for_lift{
		write string(self.name) +" waiting for lift @ "+ current_hour + " (" +h+")";
	}
	state go_alone{
		enter{
			color<-#crimson;
			if(current_hour >= start_work -(factor/4)){
//				write string(self.name) +" goint to work alone @ "+ current_hour + " (" +h+")";
			}
			if  (current_hour  >= end_work+(factor/2)){
//				write string(self.name) +" goint home alone @ "+ current_hour + " (" +h+")";
			}
		}
		if current_hour >= start_work and self.location = working_place.location{
			state<- 'working';
//			write string(self.name) +" setting state to working "+ current_hour + " (" +h+")";
		}
		if self.location = living_place.location{
			state<-'resting';
//			write string(self.name) +" setting state to resting "+ current_hour + " (" +h+")";
		}
	}
	 
	reflex move when: the_target!=nil and (state="go_alone"){ //state="go_home" or 
		path path_followed <- self goto [target::the_target, on::the_graph, return_path:: true];
		list<geometry> segments <- path_followed.segments;
		if the_target = location {
			the_target <- nil ;
		}
	}
	
	aspect base {
		draw triangle(50) color: color border: #black;
	}
}

species cars skills:[moving] control:fsm{
	list<people> give_lift_to<-nil;
	list<point> targets<-nil;
	point current_target<-nil;
	string next_people_state<-nil;
	geometry shape <- rectangle(50,100);
	
	state moving initial:true{
		if current_target=nil{
			do chose_next_target;
		}
		do move;
		transition to: stop when: (location distance_to current_target)< tolerated_distance;
	} 
	state stop{
	} 
	action chose_next_target{
		remove current_target from: targets;
		if !empty(give_lift_to){
			list<point> t <- give_lift_to collect each.location;
			add all:t to:targets;
		}
		if !empty(passenger){
			list<point> destinations <- passenger collect each.the_target;
			add all:destinations to:targets;
		}
		targets<-remove_duplicates(targets);
		
		current_target<- first( list(targets) sort_by (each distance_to (location)));
		write string(self.name)+" ("+current_hour+") chose new current_target: "+current_target +" from: "+ targets + ' - ' +state;
	}
	action move {
		write string(self.name)+" - current_target "+current_target +" location: "+ location + ' - ' +current_target distance_to location+' -'+state;
//		map general_speed_map <- road as_map (each::(each.shape.perimeter / (each.maxspeed)));
		path path_followed <- self goto [target::current_target, on::the_graph, return_path:: true,move_weights::general_speed_map];
		list<geometry> segments <- path_followed.segments;
		loop line over: segments {
			float dist <- line.perimeter;
		}
		if current_target = location {
			write string(self.name)+' ('+current_hour+') reached current_target '+location +' targets: '+ targets + ' - ' + state;
		}
	}
	reflex get_on_board when:!empty(give_lift_to) and state='stop'{
		list<people>toremove;
		loop p over: give_lift_to{
			if p.location=current_target{
				if p.the_target!=nil{
					add p to: toremove;
					capture p as:passenger{
						state<-'getting_a_lift';
						the_target<-the_target;
						color <- nil ;
						living_place <- living_place ;
						working_place <- working_place;
						start_work <-start_work;
						end_work <-end_work;
						is_driver<-is_driver;
					}
				}
			}
		}
		remove all:toremove from:give_lift_to;
		if !empty(toremove){
			write string(self.name) +' ('+current_hour+') captured '+(toremove) +' @ '+ location +' with: '+targets+ ' - ' +state;
		}
		do chose_next_target;
		state<-'moving';
	}
	reflex drop_people when:!empty(passenger) and state='stop'{
		list<people>dropped;
		string substitute_state;
		loop p over: (passenger){
			if p.the_target = current_target{
				add p to: dropped;
				if p.the_target!=location{
					substitute_state<-'go_alone';
				}else{
					substitute_state<-next_people_state;
				}
				release p in:world as:people{
					location<-myself.location;
					state<- substitute_state;
					the_target<-the_target;
					living_place <- living_place ;
					working_place <- working_place;
					start_work <- start_work;
					end_work <- end_work;
				}
			}
		}
		if !empty(dropped){
			write string(self.name) +' ('+current_hour+') dropped '+(dropped) +' @ '+ location +' with: '+targets+ ' - ' +state;
		}
		if empty(passenger){
			write string(self.name) +' ('+current_hour+') DIED';
			do die;
		}
		do chose_next_target;
		state<-'moving';
	}
	
	species passenger parent: people{
		rgb color;
		building living_place;
		building working_place;
		int start_work ;
		int end_work ;
		point the_target;
		bool is_driver;
		
		state getting_a_lift{
		}
		aspect default{
		}
	}
	aspect base{
		draw shape color: #red border: #darkred;	
	}
	
}


experiment liftToAndFromWork type: gui {
	float minimum_cycle_duration <- 0.01;
	output {
		display map type: opengl {
			graphics "world" {
				draw world.shape.contour;
			}
			species road aspect: geom refresh:true;
			species n aspect: geom3D refresh:true;
			species building aspect: base refresh:true;
			species people aspect: base refresh: true ;
			species cars aspect: base refresh: true ;
		}
	}
}