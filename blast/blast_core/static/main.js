
robots = {};
robots_plan = {};
surfaces = {};

current_map_data = null;
pixel_scale = 0.0; //pixels per meter
map_zoom = 1.0;

selected = null;
selected_type = null;
drag = null;

plan_divs = [];

///////////////////////////////////////////////////////////////////

z_orders = {};
Z_STACK = 10000; //10000 items per type
// Z_STACK * 1 = surfaces
// Z_STACK * 2 = real robots
// Z_STACK * 3 = planning

// Z_STACK * 100 = dialogs

function get_z_order(name) {
    if (z_orders[name]) {
	return z_orders[name];
    }
    var found = true;
    var z = 0;
    while (found) {
	found = false;
	z = z + 1;
	for (var name in z_orders) {
	    if (z == z_orders[name]) {
		found = true;
		break;
	    }
	}
    }
    z_orders[name] = z;
    return z;
}

///////////////////////////////////////////////////////////////////

function toFixed(value, precision) {
    var precision = precision || 0,
    neg = value < 0,
    power = Math.pow(10, precision),
    value = Math.round(value * power),
    integral = String((neg ? Math.ceil : Math.floor)(value / power)),
    fraction = String((neg ? -value : value) % power),
    padding = new Array(Math.max(precision - fraction.length, 0) + 1).join('0');
    return precision ? integral + '.' +  padding + fraction : integral;
}

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

$.deleteJSON = function(url, cb) {
    return $.ajax({type: "DELETE", url: url, dataType: 'json',
		   contentType: 'application/json',
		   data: JSON.stringify({}),
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
is_editting_world = false;
$('#edit-world').click(function() {
    if (edit_world_action !== null) {
	update_select(null, null);
	$.putJSON("/edit_world", edit_world_action, function(data) { 
	    if (!data) {
		alert("Could not edit the world.");
	    }
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
		is_editting_world = true;
		edit_world_action = false;
	    } else if (data.edit_world && data.edit_other_session) {
		$('#edit-world').html("Other user is editing world")
		    .attr("title", "Waiting for them to stop");
		is_editting_world = false;
		edit_world_action = null;
	    } else if (!data.edit_world) {
		$('#edit-world').html("Planning world")
		    .attr("title", "Click to edit actual world state");
		is_editting_world = false;
		edit_world_action = true;
	    } else {
		is_editting_world = false;
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


$('#plan-to-world').click(function() {
    $.putJSON('/plan/plan', null, function(r) {
	if (!r) {
	    alert("Planning failed!");
	    return;
	}
	if (!r[1] || !r[2]) {
	    alert("Planning failed!");
	    return;
	}
	if (selected == null && selected_type == null) {
	    update_select(null, null);
	}
    });
});
$('#plan-clear').click(function() {
    if (confirm("Delete all actions?")) {
	$.deleteJSON("/plan/plan", function() {
	    if (selected == null && selected_type == null) {
		update_select(null, null);
	    }
	});
    }
});


///////////////////////////////////////////////////////////////////


function call_feed(time) {

    $.getJSON("/feed/" + time, function(data) {
	if (!data) {
	    return;
	}

	console.log("World: " + data[1]);
	console.log("Type: " + data[2]);
	console.log("Item: " + data[3]);

	if (data[1] == null && data[2] == "plan" && data[3] == null) {
	    //Reload null select
	    if (selected_type == null && selected == null) {
		update_select(null, null);
	    }
	} else if (data[1] == null && data[2] == "edit-world" && data[3] == null) {
	    update_session();
	} else if (data[2] == "robot" && data[3] && (!data[1] || data[1] == "plan")) {
	    if (data[1] == "plan") {
		load_plan_robot(data[3], false);
	    } else {
		load_physical_robot(data[3], false);
	    }
	} else if (data[2] == "notification") {
	    var sep = data[3].indexOf(':');
	    var type = data[3].substring(0, sep);
	    var text = data[3].substring(sep + 1);
	    var text_types = {'0': "Error", '1': "Warning", '2': "Done"};
	    console.log(type);
	    console.log(text);
	    alert(text_types[type] + " -- " + text);
	} else if (data[2] == "terminate") {
	    //Just exit out, so we do not call the feed again. The server shutdown.
	    //TODO: display a message?
	    return;
	} else {
	    alert("Invalid message from server: " + data[2]);
	}
	call_feed(data[0]);
    });

}

call_feed(0);

///////////////////////////////////////////////////////////////////

$('#plan-action-div').css("z-index", Z_STACK * 100).hide();
$('#plan-action').click(function () {
    $('#plan-action-robot').html('');
    for (var r in robots) {
	$('#plan-action-robot').append('<option value=\"' + r + '\">' + r + '</option>');
    }
    $('#plan-action-robot').unbind('change').change(function() {
	$('#plan-action-type').html('');
	$('#plan-action-type').unbind('change');
	$.getJSON('/action_type/' + robots[$(this).val()].data.robot_type.name, 
		  function(data) {
		      for (var d in data) {
			  if (data[d][2]) {
			      $('#plan-action-type').append('<option value=\"' + data[d][0] + "." +
							    data[d][1] + '\">' + data[d][1] + '</option>');
			  }
		      }
		      $('#plan-action-type').append('<option disabled="disabled">----------------------</option>');
		      for (var d in data) {
			  if (!data[d][2]) {
			      $('#plan-action-type').append('<option value=\"' + data[d][0] + "." +
							    data[d][1] + '\">' + data[d][1] + '</option>');
			  }
		      }
		      $('#plan-action-type').unbind('change').change(function() {
			  $('#plan-action-items').html('');
			  $.getJSON('/action_type/' + $(this).val().replace(".", "/"),
				    function(data) {
					$('#plan-action-items').data('parameters', data.parameters);
					for (var param in data.parameters) {
					    var param_type = data.parameters[param];
					    if (param_type == "Pt") {
						$('#plan-action-items').append(param + ": x:<input id=\"plan-action-item-x-" + param
									       + "\" value=\"17.5\" size=\"10\"></input>");
						$('#plan-action-items').append(" y: <input id=\"plan-action-item-y-" + param 
									       + "\" value=\"38.4\" size=\"10\"></input>");
						$('#plan-action-items').append(" a: <input id=\"plan-action-item-a-" + param
									       + "\" value=\"-2.3\" size=\"10\"></input>");
						var map_sel = $('<select id="plan-action-item-map-' + param + '"></select>');
						for (var i in maps) {
						    var sel = ""; if (maps[i] == "clarkcenterfirstfloor") { sel = "selected=\"selected\""; }
						    map_sel.append('<option value=\"' + maps[i] + '\" ' + sel + '>' + maps[i] + '</option>');
						}
						$('#plan-action-items').append(map_sel);
						$('#plan-action-items').append("<br>");
					    } else if (param_type.indexOf("Surface:") == 0) {
						var st = param_type.split(":")[1],  sel = "selected=\"selected\"";
						var surface_sel = $('<select id="plan-action-item-surface-' + param + '"></select>');
						for (var i in surfaces) {
						    if (surfaces[i].data.type == st) {
							surface_sel.append('<option value=\"' + i + '\" ' + sel + '>' + i + '</option>');
							sel = "";
						    }
						}
						$('#plan-action-items').append(param + ": ");
						$('#plan-action-items').append(surface_sel);
						$('#plan-action-items').append("<br>");
					    } else if (param_type == "String" || param_type.indexOf("Joint:") == 0) {
						$('#plan-action-items').append(param + ": <input id=\"plan-action-item-str-" + param + "\"></input> <br>");
					    } else {
						$('#plan-action-items').append(param + ": Invalid Parameter Type: " + param_type + "<br>");
					    }
					}
				    }
				   );
		      }).trigger("change");
		  });


    }).trigger('change');
    $('#plan-action-div').show();
});

$('#plan-action-ok').click(function () {
    var parameters = $('#plan-action-items').data('parameters');
    var params = {};
    for (var param in parameters) {
	var param_type = parameters[param];
	if (param_type == "Pt") {
	    params[param] = {"x": $("#plan-action-item-x-" + param).val(),
			     "y": $("#plan-action-item-y-" + param).val(),
			     "a": $("#plan-action-item-a-" + param).val(),
			     "map": $("#plan-action-item-map-" + param).val()};
	} else if (param_type.indexOf("Surface:") == 0) {
	    params[param] = $("#plan-action-item-surface-" + param).val();
	} else if (param_type == "String" || param_type.indexOf("Joint:") == 0) {
	    params[param] = $("#plan-action-item-str-" + param).val();
	} else {
	    alert("Invalid parameter type: " + param);
	}
    }
    
    $.putJSON('/plan/plan', {"robot": $('#plan-action-robot').val(), "action": $('#plan-action-type').val(),
			     "parameters": params },
	      function(r) {
		  update_select(null, null);
		  $.getJSON('/robot/' + $('#plan-action-robot').val() + "/location", function (data) {
		      show_map(data.map);
		  });
	      });
    $('#plan-action-div').hide();
});

$('#plan-action-cancel').click(function () {
    $('#plan-action-div').hide();
});

///////////////////////////////////////////////////////////////////

maps = {};

$.getJSON( "/map", function( data ) {
    maps = data;
    for (var i in data) {
        var map = data[i];
        $('<div class="list-element">' + map + '</div>').data('map', map)
            .appendTo('#map-list').click(function() { show_map($(this).data('map')); });	
    }
});



function position_div(div, x, y, a) {
    div.css("position", "absolute");
    var tf = "rotate(" + (90 - a * 180.0 / 3.14159) + "deg)";
    div.css("transform", tf);
    div.css("-ms-transform", tf);
    div.css("-webkit-transform", tf);

    var bw = div.css("border").split(" ")[0].split("px")[0] * 1.0;

    div.css("left", x * pixel_scale - div.width() / 2 - bw);
    div.css("top", $('#map-image').height() - (y * pixel_scale + bw + div.height() / 2));
}

function screenXtoWorldX(x) {
    return (x / pixel_scale) / (map_zoom * map_zoom);
}
function screenYtoWorldY(y) {
    return (-y) / pixel_scale / (map_zoom * map_zoom) + $('#map-image').height() / pixel_scale;
}
function zoomlessScreenXtoWorldX(x) {
    return (x / pixel_scale);
}
function zoomlessScreenYtoWorldY(y) {
    return (-y) / pixel_scale + $('#map-image').height() / pixel_scale;
}


function check_robot_same(robot_name) {
    if (robots_plan[robot_name]) {
	if (same_robot(robots_plan[robot_name], robots[robot_name])) {
	    robots_plan[robot_name].div.hide();
	} else {
	    robots_plan[robot_name].div.show();
	}
    }
}

function redraw_robot(robots, robot_data, border) {
    console.log(current_map_data.map + " " + robot_data.location.map);
    if (current_map_data.map != robot_data.location.map) {
	if (robots[robot_data.name].div) { robots[robot_data.name].div.remove(); }
	if (robots[robot_data.name].halo) { robots[robot_data.name].halo.remove(); }
	return false;
    }
    
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

    if (!robots[robot_data.name]) {
	robots[robot_data.name] = {};
    }
    robots[robot_data.name].data = robot_data;
    robots[robot_data.name].div = div;
    
    if (selected == robot_data.name && (selected_type == "plan-robot" || selected_type == "robot")) {
	update_select(selected_type, robot_data.name);
    }
    return true;
};




function update_select_robot(robot, robot_dir, selected_type_r) {
    if (robot.div) {
	
    }
    if (robot.halo) {
	robot.halo.remove();
    }

    var robot_url = "/robot/";
    if (selected_type_r == "plan-robot") { robot_url = "/world/plan/robot/"; }

    robot.div.css("border", "3px solid #e67e22");
    robot.halo = $('<div style="position: absolute; color: #e67e22;">X</div>').appendTo(robot.div);
    robot.halo.css("left", (robot.div.width() / 2 - robot.halo.width() / 2) + "px");
    robot.halo.css("top", (-robot.div.height() - robot.halo.height()) / 1.5 + "px");
    
    $('#edit-robot').children().each(function(i) {
	if ($(this).attr('id') != '') {
	    $(this).remove();
	}
    });
    
    var edit_idx = 1;
    for (var pn in robot.data.robot_type.position_variables) {
	edit_idx++;
	if (edit_idx > 2) { edit_idx = 1; }
	var d = $('<div class="edit-item-' + edit_idx + '" id="' + pn 
		  + '"><div>' + pn + '</div></div>').appendTo($('#edit-robot'));
	var dt = robot.data.robot_type.position_variables[pn];
	if (dt) {
	    for (var p in dt['False'][0]) {
		var name = dt['False'][0][p];
		$('<div>' + name + ": </div>").appendTo(d).append(
		    $('<input id="' + name + '" value="' + robot.data.positions[pn][name]
		      + '" style="width: 50px;" />').keyup(function() { 
			  var joint = $(this).attr('id');
			  var pos = $(this).parent().parent().attr('id');
			  var val = $(this).val();
			  var up = false;
			  if (!isNaN(val)) {
			      if (val != robot.data.positions[pos][joint]) {
				  robot.data.positions[pos][joint] = val;
				  up = true;
			      }
			  } else {
			      if (val == 'false') {
				  if (val != robot.data.positions[pos][joint]) {
				      robot.data.positions[pos][joint] = false;
				      up = true;
				  }
			      }
			  }
			  
			  if (up) {
			      $.postJSON(robot_url + selected + "/position/" + pos, 
					 robot.data.positions[pos], function() { });
			  }
		      }));
	    }
	    
	}
    }

    $('#edit-robot-name').html(robot.data.name);
    $('#edit-robot-x').val(robot.data.location.x);
    $('#edit-robot-y').val(robot.data.location.y);
    $('#edit-robot-a').val(robot.data.location.a);
    $('#edit-robot-map').html('');
    for (var map in maps) {
	var map = maps[map];
	var sel = "";
	if (map == robot.data.location.map) { sel = "selected=\"selected\""; }
	$('#edit-robot-map').append("<option value=\"" + map + "\" " + sel + ">" + map + "</option>");
    }


    $('#edit-robot-x').unbind('keyup').keyup(function() {
	if (robot.data.location.x == 1.0 * $(this).val()) { return; }
	robot.data.location.x = 1.0 * $(this).val();
	$.postJSON(robot_url + selected + "/location", robot.data.location, function() { });
    });
    $('#edit-robot-y').unbind('keyup').keyup(function() {
	if (robot.data.location.y == 1.0 * $(this).val()) { return; }
	robot.data.location.y = 1.0 * $(this).val();
	$.postJSON(robot_url + selected + "/location", robot.data.location, function () { });
    });
    $('#edit-robot-a').unbind('keyup').keyup(function() {
	if (robot.data.location.a == 1.0 * $(this).val()) { return; }
	robot.data.location.a = 1.0 * $(this).val();
	$.postJSON(robot_url + selected + "/location", robot.data.location, function() {} );
    });
    $('#edit-robot-map').unbind('change').change(function() {
	robot.data.location.map = $(this).val();
	$.postJSON(robot_url + selected + "/location", robot.data.location,
		   function() {
		       if ($(this).val() != current_map_data.map) {
			   if (robot.div) { robot.div.hide(); }
			   if (robot.halo) { robot.halo.remove(); }
			   update_select(null, null);
		       }
		   });
    });

    $('#edit-robot-reset').unbind('click').click(function() {
	$.postJSON(robot_url + selected + "/location", 
		   robots[robot.data.name].data.location, function () { });
	for (var pos in robots[robot.data.name].data.positions) {
	    $.postJSON(robot_url + selected + "/position/" + pos, 
		       robots[robot.data.name].data.positions[pos], function () { });
	}
    });
    $('#edit-robot-exit').unbind('click').click(function() {
	update_select(null, null);
    });
    $('#edit-robot').show();    
    
    robot.halo.unbind("mousedown").mousedown(function() {
	if (drag != null) { return; }
	drag = $(this);
	$(document).unbind("mousemove").mousemove(function(event) {
	    var dx = screenXtoWorldX(event.pageX - $('#map').parent().offset().left) - robot.data.location.x;
	    var dy = screenYtoWorldY(event.pageY - $('#map').parent().offset().top) - robot.data.location.y;
	    var a = -Math.atan2(dx, dy) + Math.PI / 2;
	    robot.data.location.a = a;
	    position_div(drag.parent(), robot.data.location.x, 
			 robot.data.location.y, robot.data.location.a);
	});
	$(document).unbind("mouseup").mouseup(function() {
	    $.postJSON(robot_url + selected + "/location", robot.data.location,
		       function() { });
	    drag = null;
	    $(document).unbind("mousemove").unbind("mouseup");
	});
    });

    console.log("Update select div");
    
    robot.div.unbind("mousedown").mousedown(function() {
	console.log("Down: " + drag);
	if (drag != null) { return; }
	drag = $(this);
	$(document).unbind("mousemove").mousemove(function(event) {
	    var dx = (event.pageX - $('#map').parent().offset().left);
	    var dy = (event.pageY - $('#map').parent().offset().top);
	    robot.data.location.x = screenXtoWorldX(dx);
	    robot.data.location.y = screenYtoWorldY(dy);
	    position_div(drag, robot.data.location.x, 
			 robot.data.location.y, robot.data.location.a);
	});
	$(document).unbind("mouseup").mouseup(function() {
	    $.postJSON(robot_url + selected + "/location", robot.data.location,
		       function() {
			   $.getJSON(robot_url + selected + "?include_type=true", 
				     function(robot_data) { });
		       });
	    drag = null;
	    $(document).unbind("mousemove").unbind("mouseup");
	});
    });
}

function same_robot(r1, r2) { //visiually the same
    if (!r1 || !r2) { return r1 == r2; }
    return r1.data.location.x == r2.data.location.x
	&& r1.data.location.y == r2.data.location.y
	&& r1.data.location.a == r2.data.location.a
	&& r1.data.location.map == r2.data.location.map;
}

function load_physical_robot(robot, nuke_select) {
    $.getJSON("/robot/" + robot + "?include_type=true", function(d) {
	if (!redraw_robot(robots, d)) {
	    return;
	}
	robots[d.name].div.css("z-index", Z_STACK * 2 + get_z_order(d.name));
	robots[d.name].default_ocf = function() {
	    console.log("Click the robot");
	    if (is_editting_world) {
		update_select("edit-robot", $(this).data("robot"));
	    } else {
		if (robots_plan[d.name].data.location.map == robots[d.name].data.location.map) {
		    update_select(null, null);
		    robots_plan[d.name].div.show();
		    update_select("plan-robot", $(this).data("robot"));
		} else {
		    alert("Robot is on " + robots_plan[d.name].data.location.map + ".");
		}
	    }
	};
	robots[d.name].div.click(robots[d.name].default_ocf);
	check_robot_same(d.name);
	if (nuke_select) {
	    update_select(null, null);
	} else if (selected_type == "robot" && selected == d.name) {
	    update_select(selected_type, selected);
	}
    });
}

function load_plan_robot(robot, nuke_select) {
    $.getJSON("/world/plan/robot/" + robot + "?include_type=true", function(d) {
	if (!redraw_robot(robots_plan, d, "3px solid #2ecc71")) {
	    return;
	}
	robots_plan[d.name].div.css("z-index", Z_STACK * 3 + get_z_order(d.name));
	robots_plan[d.name].div.click(function() {
	    update_select("plan-robot", $(this).data("robot"));
	});
	check_robot_same(d.name);
	if (nuke_select) {
	    update_select(null, null);
	} else if (selected_type == "plan-robot" && selected == d.name) {
	    update_select(selected_type, selected);
	}
    });
}



function update_select(nst, ns) {
    if (selected_type == "plan-robot") {
	if (robots_plan[selected].div) {
	    if (same_robot(robots_plan[selected], robots[selected])) {
		robots_plan[selected].div.hide();
	    }
	    robots_plan[selected].div.css("border", "3px solid #2ecc71");
	    robots_plan[selected].div.unbind("mousedown");
	    robots_plan[selected].div.click(function() {
		update_select("plan-robot", $(this).data("robot"));
	    });
	}
	if (robots_plan[selected].halo) {
	    robots_plan[selected].halo.remove();
	}
    }

    if (selected_type == "edit-robot") {
	if (robots[selected].div) {
	    robots[selected].div.css("border", "0px");
	    robots[selected].div.unbind("mousedown");
	    if (!robots[selected].default_ocf) { alert("ocf sucks"); }
	    robots[selected].div.unbind("click").click(robots[selected].default_ocf);
	}
	if (robots[selected].halo) {
	    robots[selected].halo.remove();
	}
    }


    selected = ns;
    selected_type = nst;
    drag = null;
    if (selected_type == "plan-robot") {
	$('#edit-nothing').hide();
	update_select_robot(robots_plan[selected], robots_plan, "plan-robot");
    } else if (selected_type == "edit-robot") {
	$('#edit-nothing').hide();
	update_select_robot(robots[selected], robots, "edit-robot");
    } else {
	$('#edit-robot').hide();
    }

    if (selected_type == null && selected == null) {
	$('#edit-robot').hide();
	$('#edit-nothing').show();
	$('#edit-nothing').children().each(function(i) {
	    if (!$(this).attr('id')) {
		$(this).remove();
	    }
	});
	
	$.getJSON("/plan/plan", function(plan) {
	    return;
	    var edit_idx = 1;

	    $('#edit-nothing').children().each(function(i) {
		if (!$(this).attr('id')) {
		    $(this).remove();
		}
	    });
	    
	    var edit_idx = 1;
	    for (var robot in robots) {
		var robot = robots[robot];
		edit_idx++;
		if (edit_idx > 2) { edit_idx = 1; }
		$('<div class="edit-item-' + edit_idx + '">' + robot.data.name + '</div>')
		    .insertBefore($('#map-items-buffer')).data('robot', robot.data.name)
		    .click(function() {
			if (is_editting_world) {
			    update_select("edit-robot", $(this).data('robot'));
			} else {
			    update_select("plan-robot", $(this).data('robot'));
			}
		    });
	    }
	    
	    $('#edit-nothing').children().each(function(i) {
		if (!$(this).attr('id')) {
		    $(this).remove();
		}
	    });
	
	    for (var i in plan_divs) {
		plan_divs[i].remove();
	    }
	    plan_divs = [];
	    if (plan != null) {
		
		var robot_locations = {};
		for (var robot in robots) {
		    robot_locations[robot] = robots[robot].data.location;
		}

		var fixed_actions = 0;

		for (var action_i in plan[0]) {
		    var action = plan[0][action_i];

		    for (var step_i in plan[3][action[0]][action[1]].display) {
			var step = plan[3][action[0]][action[1]].display[step_i];
			if (step[0] == "line") {
			    var startl = action[2][step[1]];
			    if (step[1] == "robot.location") {
				startl = robot_locations[robot];
			    }
			    var endl = action[2][step[2]];
			    if (step[2] == "robot.location") {
				endl = robot_locations[robot];
			    }
			    if (endl && startl) {
				if (endl.map != current_map_data.map) { endl = null; }
				if (startl.map != current_map_data.map) { startl = null; }
			    } else {
				console.log("Failed to draw line:");
				console.log(step);
			    }

			    if (endl && startl) {
				var len = Math.sqrt((startl.x - endl.x) * (startl.x - endl.x) + (startl.y - endl.y) * (startl.y - endl.y));
				var ndots = Math.floor(len * 2);
				if (ndots < 2) { ndots = 2; }
				for (var dot_iter = 0; dot_iter < ndots; dot_iter++) {
				    var scale = dot_iter / (ndots - 1.0);
				    var x = (endl.x - startl.x) * scale + startl.x;
				    var y = (endl.y - startl.y) * scale + startl.y;
				    var dot = $('<div style=\"position: absolute; width: 3px; height: 3px; background-color: #'
						+ step[3] + '; "></div>').appendTo("#map");
				    position_div(dot, x, y, 0);
				    plan_divs.push(dot);
				}
			    }
			} else if (step[0] == "icon") {
			    var l = action[2];
			    if (step[1] == "robot.location") {
				l = robot_locations[robot];
			    } else {
				for (var sub in step[1].split(".")) {
				    if (typeof(l) == "string") {
					l = surfaces[l].data;
				    }
				    l = l[step[1].split(".")[sub]];
				}
			    }
			    if (l.map == current_map_data.map) {
				var dot = $('<div style=\"position: absolute; width: 5px; height: 5px;'
					    + 'background: url(\'/fspng/' + step[2] + '\');"></div>').appendTo("#map");
				var a = 0; if (step[3] == "rotate") { a = l.a; }
				position_div(dot, l.x, l.y, a);
				plan_divs.push(dot);
			    }
			} else {
			    console.log("Failed a display element:");
			    console.log(step);
			}
		    }
		    for (var step_name in plan[3][action[0]][action[1]].changes) {
			var step = plan[3][action[0]][action[1]].changes[step_name];
			if (step_name == "robot.location") {
			    robot_locations[robot] = action[2];
			    for (var sub in step.split(".")) {
				if (typeof(robot_locations[robot]) == "string") {
				    robot_locations[robot] = surfaces[robot_locations[robot]].data;
				}
				robot_locations[robot] = robot_locations[robot][step.split(".")[sub]];
			    }
			}
		    }

		    edit_idx++;
		    if (edit_idx > 2) { edit_idx = 1; }
		    var str = "";
		    for (var param in action[2]) {
			var value = action[2][param];
			if ('object' == typeof(value)) {
			    if (value.x != undefined && value.y != undefined &&
				value.a != undefined && value.map != undefined) {
				str = str + ", " + param + ":(" + toFixed(value.x, 2) + ","
				    + toFixed(value.y, 2) + "," + toFixed(value.a, 2) + "," + value.map + ")";
			    } else {
				str = str + ", " + param + ":STRANGE_OBJECT";
			    }
			} else {
			    str = str + ", " + param + ":" + value;
			}
		    }
		    var color = "";
		    if (action[3]) {
			fixed_actions++;
			color = "background-color: grey;";
		    }
		    $('<div class="edit-item-' + edit_idx + '" style="' + color + '">' + action[0] 
		      + ': ' + action[1] + str + '</div>').data("idx", action_i - fixed_actions)
			.insertBefore($('#plan-buffer')).click(function() {
			    if (confirm("Delete all actions after this one?")) {
				$.deleteJSON("/plan/plan?start=" + $(this).data("idx"), function() {
				    if (selected == null && selected_type == null) {
					update_select(null, null);
				    }
				});
			    }
			});
		}
	    }

	    if (plan != null && plan[1]) {
		$('#plan-execute').removeAttr("disabled");
		$('#plan-execute').attr("title", "");
	    } else {
		$('#plan-execute').attr("disabled", "disabled");
		$('#plan-execute').attr("title", "Cannot execute this plan because the goal world changed");
	    }
	    if (plan != null && plan[0].length > 0) {
		$('#plan-clear').removeAttr("disabled");
		$('#plan-clear').attr("title", "");
		
	    } else {
		$('#plan-clear').attr("disabled", "disabled");
		$('#plan-clear').attr("title", "Can't clear empty plan");
	    }

	});
    }
}

$('#plan-execute').click(function() {
    $.postJSON("/execute_plan/plan", null, function(result) {
	if (result !== true) {
	    alert("Bad planning result: " + result);
	}
    });
});

update_select(null, null);


function show_map(map) {
    hide_all();
    $('#map-screen').show();
    location.hash = encodeURIComponent("maps/" + map);
    
    $.getJSON("/map/" + map, function (data) {
	current_map_data = data;
	pixel_scale = data.ppm;
	$('#map-image').unbind('load').load(function() {
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
	    
	    update_select(null, null);
	    
	    $.getJSON("/robot?map=" + map, function(data) {
		for (var robot in data) {
		    load_physical_robot(data[robot], true);
		}
	    });
	    $.getJSON("/world/plan/robot?map=" + map, function(data) {
		for (var robot in data) {
		    load_plan_robot(data[robot], true);
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
				  update_select(null, null);
			      });
		}
	    });
	});
	
	$('#map-image').attr('src', '/fspng/' + data.map_file);

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



