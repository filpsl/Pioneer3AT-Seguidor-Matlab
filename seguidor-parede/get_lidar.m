function dists = get_lidar(udp_lidar, angulos_desejados, margem)
% Retorna um vetor de distâncias (medianas) para cada ângulo em angulos_desejados.
% Ângulos sem leitura válida retornam -1.
% Exemplo: dists = get_lidar(udp_lidar, [0, 90, 180], 2)

n = length(angulos_desejados);
leituras = cell(1, n);

num_pacotes = udp_lidar.NumDatagramsAvailable;

if isempty(num_pacotes) || num_pacotes == 0
    dists = -ones(1, n);
    return
end

dados_raw = read(udp_lidar, num_pacotes, "string");

for i = 1:num_pacotes
    valores = sscanf(dados_raw(i).Data, '%f, %f, %d');

    if length(valores) == 3
        angulo_atual  = valores(1);
        distancia_atual = valores(2);
        qualidade     = valores(3);

        if qualidade > 0
            for k = 1:n
                if abs(angulo_atual - angulos_desejados(k)) <= margem
                    leituras{k}(end+1) = distancia_atual;
                end
            end
        end
    end
end

dists = zeros(1, n);
for k = 1:n
    if isempty(leituras{k})
        dists(k) = -1;
    else
        m = median(leituras{k});
        dists(k) = m;
    end
end

end

