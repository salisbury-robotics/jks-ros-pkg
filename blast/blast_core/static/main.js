
///////////////////////////////////////////////////////////////////
$.postJSON = function(url, data, cb) {
    return $.ajax({type: "POST", url: url, dataType: 'json',
		   contentType: 'application/json',
		   data: JSON.stringify(data), success: cb});
};
$.putJSON = function(url, data, cb) {
    return $.ajax({type: "PUT", url: url, dataType: 'json',
		   async: false, data: JSON.stringify(data),
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

$.putJSON( "/session", {}, function(data) {
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
surfaces = {};

current_map_data = null;
pixel_scale = 20.0; //pixels per meter

function position_div(div, x, y, a) {
    div.css("position", "absolute");
    var tf = "rotate(" + (90 - a * 180.0 / 3.14159) + "deg)";
    div.css("transform", tf);
    div.css("-ms-transform", tf);
    div.css("-webkit-transform", tf);

    div.css("left", x * pixel_scale - div.width() / 2);
    div.css("top", $('#map').height() - y * pixel_scale - div.height() / 2);
}

function show_map(map) {
    hide_all();
    $('#map-screen').show();
    location.hash = encodeURIComponent("maps/" + map);
    
    $.getJSON("/map/" + map, function (data) {
	current_map_data = data;
	$('#map-image').attr('src', '/fspng/' + data.map_file);

	for (var robot in robots) { 
	    robots[robot].div.remove();
	}
	for (var surface in surfaces) { 
	    for (var i in surfaces[surface].divs) { 
		surfaces[surface].divs[i].remove();
	    }
	}

	$.getJSON("/robot", function(data) {
	    for (var robot in data) {
		var robot = data[robot];
		$.getJSON("/robot/" + robot,
			  function(robot_data) {
			      if (robot_data.location.map != map) return;
			      var div = $('<div style="border: 1px solid black;">R</div>').appendTo('#map');
			      position_div(div, robot_data.location.x, 
					   robot_data.location.y, robot_data.location.a);
			      robots[robot] = {'data': robot_data,
					       'div': div};
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



