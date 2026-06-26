function status = teleportModel(worldName, modelName, x, y, z, yaw)
% Teleporta o modelo para uma posição.
    qw = cos(yaw/2);
    qz = sin(yaw/2);
    
    req = sprintf(['name: "%s", position: {x: %g, y: %g, z: %g}, ' ...
                   'orientation: {x: 0, y: 0, z: %g, w: %g}'], ...
                   modelName, x, y, z, qz, qw);
    
    cmd = sprintf(['gz service -s /world/%s/set_pose ' ...
                   '--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean ' ...
                   '--timeout 3000 --req ''%s'''], worldName, req);

    [ok, out] = system(cmd);
    status = (ok == 0) && contains(out, 'true');
    if ~status
        warning('Falha ao teleportar objeto: \n%s', out);
    end

end