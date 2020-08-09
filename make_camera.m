function camera = make_camera(x, y, range, color)
    
    % Initialize the camera
    camera.x = x;
    camera.y = y;
    camera.range = range;
    camera.color = color;
    
    % Camera patch
    fig_coords = [x-1, x+1, x; y, y, y+2];
    camera.fig_coords = fig_coords;
    camera.h = patch(fig_coords(1,:), fig_coords(2,:), color);
end
