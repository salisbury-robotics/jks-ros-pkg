
///////////////////////////////////////////////////////////////////
$.postJSON = function(url, data, cb) {
    return $.ajax({type: "POST", url: url, dataType: 'json',
		   contentType: 'application/json',
		   data: JSON.stringify(data), success: cb});
};
$.putJSON = function(url, data, cb) {
    return $.ajax({type: "PUT", url: url, dataType: 'json',
		   contentType: 'application/json',
		   data: JSON.stringify(data),
		   success: cb});
};


///////////////////////////////////////////////////////////////////
//Support reload.
old_hash_value = location.hash; 
post_hash = function() {
    lh = old_hash_value;
    var lh = decodeURIComponent(lh);
    if (lh.indexOf("#maps/") == 0) {
	show_map(lh.split("/")[1]);
    }
}

function hide_all() {
    location.hash = "";
    $('#login-screen').hide();
    $('#primary-screen').hide();
    $('#map-screen').hide();
}
hide_all();
$('#login-screen').show();

edit_world_action = null;
$('#edit-world').click(function() {
    if (edit_world_action !== null) {
	$.putJSON("/edit_world", edit_world_action, function(data) { 
	    update_session();
	});
    }
});

function update_session() {
    $.getJSON("/session", function(data) {
	if (data) {
	    if (data.edit_world && !data.edit_other_session) {
		$('#edit-world').html("Editing world")
		    .attr("title", "Click to switch to planning");
		edit_world_action = false;
	    } else if (data.edit_world && data.edit_other_session) {
		$('#edit-world').html("Other user is editing world")
		    .attr("title", "Waiting for them to stop");
		edit_world_action = null;
	    } else if (!data.edit_world) {
		$('#edit-world').html("Planning world")
		    .attr("title", "Click to edit actual world state");
		edit_world_action = true;
	    } else {
		$('#edit-world').html("Unable to edit world")
		    .attr("title", "You do not have permission to edit the world");
		edit_world_action = null;
	    }
	    $('#edit-world').toggleClass("edit-world-me", data.edit_world && !data.edit_other_session);
	    $('#edit-world').toggleClass("edit-world-other", data.edit_world && data.edit_other_session);
	} else {
	    console.log("Session timed out");
	}
    }); 
}

$.putJSON( "/session", {}, function(data) {
    //Prevent the session from timing out
    update_session();
    setInterval(update_session, SESSION_TIMEOUT * 0.9 * 1000.0);


    $.putJSON( "/world", "plan", function(data) {
	$.postJSON("/world/plan/robot/stair4/location", 
		   {"x": 20, "y": 40, "a": 0, "map": "clarkcenterfirstfloor"},
		   function(data2) {
		       $.getJSON( "/world/plan/robot/stair4/location", function(data2) {
			   console.log(data2);
		       });

		   });
    });

    hide_all();
    $('#primary-screen').show();
    post_hash();
});

$('#back').click(function() { hide_all(); $('#primary-screen').show(); });

///////////////////////////////////////////////////////////////////

$.getJSON( "/map", function( data ) {
    for (var i in data) {
        var map = data[i];
        $('<div class="list-element">' + map + '</div>').data('map', map)
            .appendTo('#map-list').click(function() { show_map($(this).data('map')); });	
    }
});


robots = {};
robots_plan = {};
surfaces = {};

current_map_data = null;
pixel_scale = 0.0; //pixels per meter
map_zoom = 1.0;

function position_div(div, x, y, a) {
    div.css("position", "absolute");
    var tf = "rotate(" + (90 - a * 180.0 / 3.14159) + "deg)";
    div.css("transform", tf);
    div.css("-ms-transform", tf);
    div.css("-webkit-transform", tf);

    div.css("left", x * pixel_scale - div.width() / 2);
    div.css("top", $('#map-image').height() - (y * pixel_scale + div.height() / 2));
}

function screenXtoWorldX(x) {
    return (x / pixel_scale) / (map_zoom * map_zoom);
}
function screenYtoWorldY(y) {
    return (-y) / pixel_scale / (map_zoom * map_zoom) + $('#map-image').height() / pixel_scale;
}


function redraw_robot(robots, robot_data, border) {
    var rd = robot_data.robot_type.display;
    var position_flags = [];
    for (var pos_name in robot_data.robot_type.position_variables) {
	var pos_meta = robot_data.robot_type.position_variables[pos_name];
	var pos = robot_data.positions[pos_name];
	for (var pose in pos_meta) {
	    if (pose === "False") { continue; }
	    var good = true;
	    for (var idx in pos_meta.False[0]) {
		var joint = pos[pos_meta.False[0][idx]];
		var tol = pos_meta.False[1][idx];
		var target = pos_meta[pose][idx];
		if (target !== false && joint !== false) {
		    if (joint > target + tol || joint < target - tol) {
			good = false;
			break;
		    }
		} else if (target !== false && joint === false) {
		    good = false;
		    break;
		}
	    }
	    if (good) { 
		position_flags.push(pos_name + "=" + pose);
	    }
	}
    }

    
    var images = [];
    for (var image_name in rd) {
	var sp = image_name.split(",");
	if (sp[0] == "image") {
	    var good = true;
	    for (var name in sp) {
		var name = sp[name];
		if (name == "image") { continue; }
		if ($.inArray(name, position_flags) < 0) {
		    good = false;
		    break;
		}
	    }
	    if (good) {
		images.push(rd[image_name]);
	    }
	}
    }
    
    if (images != []) {
	images.sort(function(a,b){return b.priority-a.priority});
	image = images[0];
    } else {
	alert("Error! " + robot_data.name + " has no valid images!");
    }
    
    
    var div = $('<div style="width: ' + rd.width * pixel_scale + 'px; height: ' 
		+ rd.height * pixel_scale  + 'px; background: url('
		+ '\'/fspng/' + image.image + '\'); background-size: ' 
		+ rd.width * pixel_scale + 'px; height: ' 
		+ rd.height * pixel_scale  + 'px;" title="'
		+ robot_data.name + '"> </div>')
	.data("robot", robot_data.name).appendTo('#map');
    if (border) {
	div.css("border", border);
    }
    position_div(div, robot_data.location.x, 
		 robot_data.location.y, robot_data.location.a);

    if (robots[robot_data.name]) {
	if (robots[robot_data.name].div) {
	    robots[robot_data.name].div.remove();
	}
    }

    robots[robot_data.name] = {'data': robot_data, 'div': div};
};

selected = null;
selected_type = null;
drag = null;
function update_select(nst, ns) {
    if (selected_type == "robot") {
	if (robots_plan[selected].div) {
	    robots_plan[selected].div.css("border", "3px solid #e67e22");
	}
	if (robots_plan[selected].halo) {
	    robots_plan[selected].halo.remove();
	}
    }
    selected = ns;
    selected_type = nst;
    drag = null;
    if (selected_type == "robot") {
	if (robots_plan[selected].div) {
	    
	}
	if (robots_plan[selected].halo) {
	    robots_plan[selected].halo.remove();
	}
	robots_plan[selected].div.css("border", "3px solid #e67e22");
	robots_plan[selected].halo = $('<div style="position: absolute; color: #e67e22;">X</div>').appendTo(robots_plan[selected].div);
	robots_plan[selected].halo.css("left", (robots_plan[selected].div.width() / 2 - robots_plan[selected].halo.width() / 2) + "px");
	robots_plan[selected].halo.css("top", (-robots_plan[selected].div.height() - robots_plan[selected].halo.height()) / 1.5 + "px");
	
	robots_plan[selected].halo.unbind("mousedown").mousedown(function() {
	    if (drag != null) { return; }
	    drag = $(this);
	    $(document).unbind("mousemove").mousemove(function(event) {
		var centerp = $('<div style="width: 0px; height: 0px; position: absolute;'
				+ 'left: ' + drag.parent().width() / 2 + 'px; top: ' 
				+ drag.parent().height() / 2 + ';"></div>').appendTo(drag.parent());
		var dx = (event.pageX - centerp.offset().left);
		var dy = (event.pageY - centerp.offset().top);
		centerp.remove();
		var a = Math.atan2(dx, dy) - Math.PI / 2;
		robots_plan[selected].data.location.a = a;
		position_div(drag.parent(), 
			     robots_plan[selected].data.location.x, 
			     robots_plan[selected].data.location.y,
			     robots_plan[selected].data.location.a);
	    });
	    $(document).unbind("mouseup").mouseup(function() {
		$.postJSON("/world/plan/robot/" + selected + "/location", robots_plan[selected].data.location,
			   function() {
			       $.getJSON("/world/plan/robot/" + selected + "?include_type=true", 
					 function(robot_data) {
					     redraw_robot(robots_plan, robot_data, "3px solid #e67e22");
					     update_select("robot", robot_data.name);
					     });
			   });
		drag = null;
		$(document).unbind("mousemove").unbind("mouseup");
	    });
	});

	robots_plan[selected].div.unbind("mousedown").mousedown(function() {
	    if (drag != null) { return; }
	    drag = $(this);
	    $(document).unbind("mousemove").mousemove(function(event) {
		var dx = (event.pageX - $('#map').parent().offset().left);
		var dy = (event.pageY - $('#map').parent().offset().top);
		robots_plan[selected].data.location.x = screenXtoWorldX(dx);
		robots_plan[selected].data.location.y = screenYtoWorldY(dy);
		position_div(drag, 
			     robots_plan[selected].data.location.x, 
			     robots_plan[selected].data.location.y,
			     robots_plan[selected].data.location.a);
	    });
	    $(document).unbind("mouseup").mouseup(function() {
		$.postJSON("/world/plan/robot/" + selected + "/location", robots_plan[selected].data.location,
			   function() {
			       $.getJSON("/world/plan/robot/" + selected + "?include_type=true", 
					 function(robot_data) {
					     redraw_robot(robots_plan, robot_data, "3px solid #e67e22");
					     update_select("robot", robot_data.name);
					     });
			   });
		drag = null;
		$(document).unbind("mousemove").unbind("mouseup");
	    });
	});

    }
}


function show_map(map) {
    hide_all();
    $('#map-screen').show();
    location.hash = encodeURIComponent("maps/" + map);
    
    $.getJSON("/map/" + map, function (data) {
	current_map_data = data;
	pixel_scale = data.ppm;
	$('#map-image').attr('src', '/fspng/' + data.map_file);

	for (var robot in robots) { 
	    robots[robot].div.remove();
	}
	for (var robot in robots_plan) { 
	    robots_plan[robot].div.remove();
	}
	for (var surface in surfaces) { 
	    for (var i in surfaces[surface].divs) { 
		surfaces[surface].divs[i].remove();
	    }
	}

	$.getJSON("/robot?map=" + map, function(data) {
	    for (var robot in data) {
		var robot = data[robot];
		$.getJSON("/robot/" + robot + "?include_type=true", function(d) {
		    redraw_robot(robots, d);
		});
	    }
	});
	$.getJSON("/world/plan/robot?map=" + map, function(data) {
	    for (var robot in data) {
		var robot = data[robot];
		$.getJSON("/world/plan/robot/" + robot + "?include_type=true", function(d) {
		    redraw_robot(robots_plan, d, "3px solid #2ecc71");
		    robots_plan[d.name].div.click(function() {
			update_select("robot", $(this).data("robot"));
		    });
		});
	    }
	});
	
	$.getJSON("/surface", function(data) {
	    for (var surface in data) {
		var surface = data[surface];
		$.getJSON("/surface/" + surface,
			  function(surface_data) {
			      var divs = [];
			      for (var ln in surface_data.locations) {
				  var l = surface_data.locations[ln];
				  if (l.map == map) {
				      var div = $('<div style="width: 20px; height: 20px; background: url('
						  + '\'/static/def_arrow.png\'); background-size: 20px 20px;" title="'
						  + surface_data.name + "." + ln + '"> </div>').appendTo('#map');
				      position_div(div, l.x, l.y, l.a);
				      divs.push(div);
				  }
			      }
			      surfaces[surface_data.name] = {'data': surface_data, 'divs': divs};
			  });
	    }
	});
    });
}


///////////////////////////////////////////////////////////////////

map_zoom = 1.0;
function update_zoom() {
    $('#map').css('zoom', map_zoom * 100.0 + '%');
    $('#map').css('-moz-transform', 'scale(' + map_zoom + ')');
    $('#map').css('-webkit-transform', 'scale(' + map_zoom + ')');
}
$('#zoom-in').click(function() { map_zoom *= 1.3; update_zoom(); });
$('#zoom-out').click(function() { map_zoom /= 1.3; update_zoom(); });



