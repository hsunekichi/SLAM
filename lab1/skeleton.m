%-------------------------------------------------------------------------
% University of Zaragoza
%
% Author:  J. Neira
%-------------------------------------------------------------------------
% SLAM for Karel the robot in 1D
%-------------------------------------------------------------------------

clear all; % varialbes
close all; % figures
randn('state', 1); % always use same random number sequence
rand('state', 1); % always use same random number sequence
format long

%-------------------------------------------------------------------------
% configuration parameters

global config;

% display correlation matrix and pause at every step
config.step_by_step = 0;

%number of robot motions for each local map
config.steps_per_map = 1000;

% figure counter (to always plot a new figure)
config.fig = 0;
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% 1D world characteristics

global world;

% In the absolute W reference, these will not change during program execution
world.true_point_locations = [0.5:1:100000]';

% In the absolute W reference, this will change as the robot moves
world.true_robot_location = 0;

%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% robot characteristics

global robot;

%    factor_x: fraction of the motion that constitutes odometry error
%     true_uk: ground true robot motion per step

robot.factor_x = 0.1; % ten percent
robot.true_uk = 1; % each true motion is 1m
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% sensor characteristics

global sensor;

%     factor_z: measurement error is this fraction of the distance
%    range_min: minimum sensor range
%    range_max: maximum sensor range

sensor.factor_z = 0.01; % one percent
sensor.range_min = 0;
sensor.range_max = 2;
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% all map details

global map;

%            R0: absolute location of base reference for map
%         hat_x: estimated robot and feature locations
%         hat_P: robot and feature covariance matrix
%             n: number of features in map
%        true_x: true robot and feature location (with respect to R0)
%      true_ids: true label of features in map (according to world)
%  stats.true_x: true robotlocation wrt R0 for the whole trajectory
% stats.error_x: robotlocation error at every step of the trajectory
% stats.sigma_x: robot location uncertainty at every step of the trajectory
%  stats.cost_t: computational cost (elapsed time) for each step
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% measurements (not global, but this is the structure)

%            z_k: measured distance to all visible features
%            R_k: covariance of z_k
%          ids_f: true ids of the ones that are already in the map
%          ids_n: true ids of the ones that new to the map
%        z_pos_f: positions in z_r of ids_f
%        z_pos_n: positions in z_r of ids_n
%        x_pos_f: positions in hat_x of ids_f
%            z_f: z_k(ids_f), measurements to features already in the map
%            R_f: R_k(ids_f), meas. cov. to features already in the map
%            z_n: z_k(ids_n), measurements to features new to the map
%            R_n: R_k(ids_n), Meas. cov. to features new to the map
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% BEGIN
%-------------------------------------------------------------------------

[map] = Kalman_filter_slam (map, config.steps_per_map);

display_map_results (map);

%-------------------------------------------------------------------------
% END
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% Kalman_filter_slam
%-------------------------------------------------------------------------

function[map] = Kalman_filter_slam (map, steps)

global world;

% initial ground truth of hat_x 
map.true_x = zeros(steps);
map.true_x = [0];

% initial number of features
map.n = 0;

% initial map state and covariance
map.R0 = world.true_robot_location;

map.hat_x = zeros(steps);
map.hat_x = [0];

map.hat_P = zeros(steps,steps);
map.hat_P = [0];

% feature ids, for robot = 0
map.true_ids = [0];

% useful statistics
map.stats.true_x = [0];
map.stats.error_x = [];
map.stats.sigma_x = [];
map.stats.cost_t = [];

for k = 0:steps
    
    tstart = tic;
    
    if k > 0
        [map] = compute_motion (map);
    end
    
    % get measurements
    [measurements] = get_measurements_and_data_association(map);
    
    % seen features already in the map? KF update!
    if not(isempty(measurements.z_f))
        [map] = update_map (map, measurements);
    end
    
    % some new features?
    if not(isempty(measurements.z_n))
        [map] = add_new_features (map, measurements);
    end
    
    % record statistics
    map.stats.error_x = [map.stats.error_x; (map.stats.true_x(end) - map.hat_x(1))];
    map.stats.sigma_x = [map.stats.sigma_x; sqrt(map.hat_P(1,1))];
    map.stats.cost_t = [map.stats.cost_t; toc(tstart)];
    
end
end

%-------------------------------------------------------------------------
% compute_motion
%-------------------------------------------------------------------------

function [map] = compute_motion (map)

global config;
global robot;
global world;

% move robot and update ground truth robot location
world.true_robot_location = world.true_robot_location + robot.true_uk;

% odometry error
sigma_xk = robot.factor_x * robot.true_uk;
error_k = randn(1)*sigma_xk;

% estimated motion
map.hat_x(1) = map.hat_x(1) + robot.true_uk + error_k;
map.hat_P(1,1) = map.hat_P(1,1) + sigma_xk^2;

% update ground truth for the current map
map.true_x(1) = map.true_x(1) + robot.true_uk;
map.stats.true_x = [map.stats.true_x; map.stats.true_x(end) + robot.true_uk];

if config.step_by_step
    fprintf('Move to %d...\n',map.true_x(1));
    plot_correlation(map.hat_P);
    pause
end
end

%-------------------------------------------------------------------------
% get_measurements
%-------------------------------------------------------------------------

function [measurements] = get_measurements_and_data_association(map)

global world;
global sensor;

distances = world.true_point_locations - world.true_robot_location;
visible_ids = find((distances >= sensor.range_min) & (distances <= sensor.range_max));
visible_d = distances(visible_ids);
n_visible = length(visible_ids);

% sensor error
sigma_z = sensor.factor_z * visible_d;
error_z = randn(n_visible, 1) .* sigma_z;

measurements.z_k = visible_d + error_z;
measurements.R_k = diag(sigma_z.^2);

% data association
measurements.ids_f = intersect(map.true_ids, visible_ids);
measurements.ids_n = setdiff(visible_ids, map.true_ids);
measurements.z_pos_f = find(ismember(visible_ids, measurements.ids_f));
measurements.z_pos_n = find(ismember(visible_ids, measurements.ids_n));
measurements.x_pos_f = find(ismember(map.true_ids, visible_ids));

%features already in the map
measurements.z_f = measurements.z_k(measurements.z_pos_f);
measurements.R_f = measurements.R_k(measurements.z_pos_f,measurements.z_pos_f);

%new features
measurements.z_n = measurements.z_k(measurements.z_pos_n);
measurements.R_n = measurements.R_k(measurements.z_pos_n,measurements.z_pos_n);

end

%-------------------------------------------------------------------------
% update_map
%-------------------------------------------------------------------------

function[map] = update_map (map, measurements)

global config;

% DO SOMETHING HERE!
% You need to compute H_k, y_k, S_k, K_k and update map.hat_x and map.hat_P

if config.step_by_step
    fprintf('Updape map for last %d features...\n',length(measurements.ids_f));
    plot_correlation(map.hat_P);
    pause
end
end

%-------------------------------------------------------------------------
% add_new_features
%-------------------------------------------------------------------------

function [map] = add_new_features (map, measurements)

global config;
global world;

% DO SOMETHING HERE!
% update map.hat_x, map.hat_P, map.true_ids, map.true_x, map.n

if config.step_by_step
    fprintf('Add %d new features...\n',length(measurements.ids_n));
    plot_correlation(map.hat_P);
    pause
end
end

%-------------------------------------------------------------------------
% display_results
%-------------------------------------------------------------------------

function  display_map_results (map)

global config;

config.fig = config.fig + 1;
figure(config.fig);
axis([0 length(map.hat_x) -2*max(sqrt(diag(map.hat_P))) 2*max(sqrt(diag(map.hat_P)))]);
grid on;
hold on;
plot(map.true_ids, map.hat_x-map.true_x, 'ro','Linewidth', 2);
plot(map.true_ids, 2*sqrt(diag(map.hat_P)), 'b+','Linewidth', 2);
plot(map.true_ids, -2*sqrt(diag(map.hat_P)), 'b+','Linewidth', 2);
xlabel('Feature number (robot = 0)');
ylabel('meters (m)');
title('Map estimation error + 2sigma bounds');

config.fig = config.fig + 1;
figure(config.fig);
plot_correlation(map.hat_P);
title(sprintf('Correlation matrix of size %d', size(map.hat_P,1)));

config.fig = config.fig + 1;
figure(config.fig);
grid on;
hold on;
plot(map.stats.true_x, map.stats.cost_t,'r-','Linewidth',2);
xlabel('step');
ylabel('seconds');
title('Cost per step');

config.fig = config.fig + 1;
figure(config.fig);
grid on;
hold on;
plot(map.stats.true_x,cumsum(map.stats.cost_t),'r-','Linewidth',2);
xlabel('step');
ylabel('seconds');
title('Cumulative cost');

config.fig = config.fig + 1;
figure(config.fig);
axis([0 map.stats.true_x(end) -2*max(map.stats.sigma_x) 2*max(map.stats.sigma_x)]);
grid on;
hold on;
plot(map.stats.true_x, map.stats.error_x,'r-','Linewidth',2);
plot(map.stats.true_x, 2*map.stats.sigma_x,'b-','Linewidth',2);
plot(map.stats.true_x,-2*map.stats.sigma_x,'b-','Linewidth',2);
xlabel('meters (m)');
ylabel('meters (m)');
title('Robot estimation error + 2sigma bounds');

end

%-------------------------------------------------------------------------
% plot_correlation
%-------------------------------------------------------------------------

function plot_correlation(P)

ncol=256 ;

% hot cold colormap
cmap=hsv2rgb([linspace(2/3, 0, ncol)' 0.9*ones(ncol,1) ones(ncol,1)]) ;
cmap(1,:)=[0 0 0] ;
colormap(cmap) ;

corr = correlation(P);
imagesc(abs(corr), [0 1]) ;  % no sign

axis image;
colorbar ;

end

%-------------------------------------------------------------------------
% correlation
%-------------------------------------------------------------------------

function Corr=correlation(Cov)

sigmas = sqrt(diag(Cov));
Corr = diag(1./sigmas)*Cov*diag(1./sigmas);

end
%------------------------------------------------- ------------------------
% Universidad de Zaragoza
%
% Autor: J. Neira
%------------------------------------------------- ------------------------
% SLAM para Karel el robot en 1D
%------------------------------------------------- ------------------------

borrar todo; % variables
cerrar todo; % cifras
randn('estado', 1); % siempre usa la misma secuencia de números aleatorios
rand('estado', 1); % siempre usa la misma secuencia de números aleatorios
formato largo

%------------------------------------------------- ------------------------
% parámetros de configuración

configuración global;

% muestra la matriz de correlación y hace una pausa en cada paso
config.paso_a_paso = 0;

%número de movimientos del robot para cada mapa local
config.steps_per_map = 1000;

Contador de % de cifras (para trazar siempre una nueva cifra)
config.fig = 0;
%------------------------------------------------- ------------------------

%------------------------------------------------- ------------------------
% Características del mundo 1D

mundo global;

% En la referencia W absoluta, estos no cambiarán durante la ejecución del programa.
world.true_point_locations = [0.5:1:100000]';

% En la referencia W absoluta, esto cambiará a medida que el robot se mueva.
world.true_robot_location = 0;

%------------------------------------------------- ------------------------

%------------------------------------------------- ------------------------
% características del robot

robot global;

% factor_x: fracción del movimiento que constituye error de odometría
% true_uk: movimiento real del robot en tierra por paso

robot.factor_x = 0,1; % diez por ciento
robot.true_uk = 1; % cada movimiento verdadero es de 1m
%------------------------------------------------- ------------------------

%------------------------------------------------- ------------------------
% características del sensor

sensor global;

% factor_z: el error de medición es esta fracción de la distancia
% range_min: rango mínimo del sensor
% range_max: rango máximo del sensor

sensor.factor_z = 0,01; % uno por ciento
sensor.range_min = 0;
sensor.range_max = 2;
%------------------------------------------------- ------------------------

%------------------------------------------------- ------------------------
% todos los detalles del mapa

mapa global;

% R0: ubicación absoluta de la referencia base para el mapa
% hat_x: ubicaciones estimadas de robots y funciones
% hat_P: matriz de covarianza de características y robots
% n: número de entidades en el mapa
% true_x: ubicación verdadera del robot y de la característica (con respecto a R0)
% true_ids: etiqueta verdadera de las características en el mapa (según el mundo)
% stats.true_x: verdadera ubicación del robot con R0 para toda la trayectoria
% stats.error_x: error de ubicación del robot en cada paso de la trayectoria
% stats.sigma_x: incertidumbre de la ubicación del robot en cada paso de la trayectoria
% stats.cost_t: costo computacional (tiempo transcurrido) para cada paso
%------------------------------------------------- ------------------------

%------------------------------------------------- ------------------------
% de mediciones (no globales, pero esta es la estructura)

% z_k: distancia medida a todas las características visibles
% R_k: covarianza de z_k
% ids_f: ids verdaderos de los que ya están en el mapa
% ids_n: identificadores verdaderos de los nuevos en el mapa
% z_pos_f: posiciones en z_r de ids_f
% z_pos_n: posiciones en z_r de ids_n
% x_pos_f: posiciones en hat_x de ids_f
% z_f: z_k(ids_f), mediciones de entidades que ya están en el mapa
% R_f: R_k(ids_f), med. cov. a funciones que ya están en el mapa
% z_n: z_k(ids_n), mediciones de características nuevas en el mapa
% R_n: R_k(ids_n), Medida. cov. a características nuevas en el mapa
%------------------------------------------------- ------------------------

%------------------------------------------------- ------------------------
% COMENZAR
%------------------------------------------------- ------------------------

[mapa] = Kalman_filter_slam (mapa, config.steps_per_map);

display_map_results (mapa);

%------------------------------------------------- ------------------------
% FIN
%------------------------------------------------- ------------------------

%------------------------------------------------- ------------------------
% Kalman_filter_slam
%------------------------------------------------- ------------------------

función[mapa] = Kalman_filter_slam (mapa, pasos)

mundo global;

% de verdad fundamental inicial de hat_x
map.true_x = ceros(pasos);
mapa.true_x = [0];

% número inicial de características
mapa.n = 0;

% estado inicial del mapa y covarianza
mapa.R0 = mundo.true_robot_location;

map.hat_x = ceros(pasos);
mapa.hat_x = [0];

map.hat_P = ceros(pasos,pasos);
mapa.hat_P = [0];

% identificadores de funciones, para robot = 0
mapa.true_ids = [0];

% estadísticas útiles
mapa.stats.true_x = [0];
map.stats.error_x = [];
mapa.stats.sigma_x = [];
mapa.stats.cost_t = [];

para k = 0:pasos

tstart = tic;

si k > 0
[mapa] = Compute_motion (mapa);
fin

% obtener medidas
[medidas] = get_medidas_and_data_association(mapa);

¿Porcentaje de características vistas que ya están en el mapa? ¡Actualización de KF!
si no (está vacío (medidas.z_f))
[mapa] = update_map (mapa, medidas);
fin

% algunas características nuevas?
si no (está vacío (medidas.z_n))
[mapa] = add_new_features (mapa, medidas);
fin

% estadísticas de registro
map.stats.error_x = [map.stats.error_x; (map.stats.true_x(end) - map.hat_x(1))];
map.stats.sigma_x = [map.stats.sigma_x; sqrt(map.hat_P(1,1))];
mapa.stats.cost_t = [mapa.stats.cost_t; toc(tinicio)];

fin
fin

%------------------------------------------------- ------------------------
% cálculo_movimiento
%------------------------------------------------- ------------------------

función [mapa] = Compute_motion (mapa)

configuración global;
robot global;
mundo global;

% mover el robot y actualizar la ubicación del robot de verdad terrestre
world.true_robot_location = world.true_robot_location + robot.true_uk;

% error de odometría
sigma_xk = robot.factor_x * robot.true_uk;
error_k = randn(1)*sigma_xk;

% de movimiento estimado
mapa.hat_x(1) = mapa.hat_x(1) + robot.true_uk + error_k;
mapa.hat_P(1,1) = mapa.hat_P(1,1) + sigma_xk^2;

% de actualización de la verdad del terreno para el mapa actual
mapa.true_x(1) = mapa.true_x(1) + robot.true_uk;
map.stats.true_x = [map.stats.true_x; map.stats.true_x(fin) + robot.true_uk];

si config.paso_a_paso
fprintf('Mover a %d...\n',map.true_x(1));
plot_correlation(mapa.hat_P);
pausa
fin
fin

%------------------------------------------------- ------------------------
% obtener_medidas
%------------------------------------------------- ------------------------

función [medidas] = get_medidas_y_asociación_de_datos(mapa)

mundo global;
sensor global;

distancias = world.true_point_locations - world.true_robot_location;
visible_ids = buscar((distancias >= sensor.range_min) & (distancias <= sensor.range_max));
visible_d = distancias(visible_ids);
n_visible = longitud(visible_ids);

% de error del sensor
sigma_z = sensor.factor_z * visible_d;
error_z = randn(n_visible, 1) .* sigma_z;

mediciones.z_k = visible_d + error_z;
mediciones.R_k = diag(sigma_z.^2);

% asociación de datos
mediciones.ids_f = intersect(map.true_ids, visible_ids);
mediciones.ids_n = setdiff(visible_ids, map.true_ids);
mediciones.z_pos_f = buscar(ismember(visible_ids, mediciones.ids_f));
mediciones.z_pos_n = buscar(ismember(visible_ids, mediciones.ids_n));
mediciones.x_pos_f = buscar(ismember(map.true_ids, visible_ids));

%características que ya están en el mapa
mediciones.z_f = mediciones.z_k(mediciones.z_pos_f);
medidas.R_f = medidas.R_k(medidas.z_pos_f,medidas.z_pos_f);

%nuevas características
mediciones.z_n = mediciones.z_k(mediciones.z_pos_n);
medidas.R_n = medidas.R_k(medidas.z_pos_n,medidas.z_pos_n);

fin

%------------------------------------------------- ------------------------
% mapa_actualización
%------------------------------------------------- ------------------------

function[mapa] = update_map (mapa, medidas)

configuración global;

% ¡HAGA ALGO AQUÍ!
% Necesita calcular H_k, y_k, S_k, K_k y actualizar map.hat_x y map.hat_P

si config.paso_a_paso
fprintf('Actualizar mapa para las últimas %d características...\n',length(measurements.ids_f));
plot_correlation(mapa.hat_P);
pausa
fin
fin

%------------------------------------------------- ------------------------
% agregar_nuevas_características
%------------------------------------------------- ------------------------

función [mapa] = add_new_features (mapa, medidas)

configuración global;
mundo global;

% ¡HAGA ALGO AQUÍ!
% actualización map.hat_x, map.hat_P, map.true_ids, map.true_x, map.n

si config.paso_a_paso
fprintf('Agregar %d nuevas funciones...\n',length(measurements.ids_n));
plot_correlation(mapa.hat_P);
pausa
fin
fin

%------------------------------------------------- ------------------------
% mostrar_resultados
%------------------------------------------------- ------------------------

función display_map_results (mapa)

configuración global;

config.fig = config.fig + 1;
figura(config.fig);
eje([0 longitud(map.hat_x) -2*max(sqrt(diag(map.hat_P))) 2*max(sqrt(diag(map.hat_P)))]);
rejilla encendida;
esperar;
plot(map.true_ids, map.hat_x-map.true_x, 'ro', 'Linewidth', 2);
plot(map.true_ids, 2*sqrt(diag(map.hat_P)), 'b+','Linewidth', 2);
plot(map.true_ids, -2*sqrt(diag(map.hat_P)), 'b+','Linewidth', 2);
xlabel('Número de característica (robot = 0)');
ylabel('metros (m)');
title('Error de estimación del mapa + límites de 2sigma');

config.fig = config.fig + 1;
figura(config.fig);
plot_correlation(mapa.hat_P);
title(sprintf('Matriz de correlación de tamaño %d', size(map.hat_P,1)));

config.fig = config.fig + 1;
figura(config.fig);
rejilla encendida;
esperar;
plot(map.stats.true_x, map.stats.cost_t,'r-','Linewidth',2);
xlabel('paso');
ylabel('segundos');
título('Costo por paso');

config.fig = config.fig + 1;
figura(config.fig);
rejilla encendida;
esperar;
plot(map.stats.true_x,cumsum(map.stats.cost_t),'r-','Linewidth',2);
xlabel('paso');
ylabel('segundos');
título('Costo acumulado');

config.fig = config.fig + 1;
figura(config.fig);
eje([0 map.stats.true_x(end) -2*max(map.stats.sigma_x) 2*max(map.stats.sigma_x)]);
rejilla encendida;
esperar;
plot(map.stats.true_x, map.stats.error_x,'r-','Linewidth',2);
plot(map.stats.true_x, 2*map.stats.sigma_x,'b-','Linewidth',2);
plot(map.stats.true_x,-2*map.stats.sigma_x,'b-','Linewidth',2);
xlabel('metros (m)');
ylabel('metros (m)');
title('Error de estimación del robot + límites de 2sigma');

fin

%------------------------------------------------- ------------------------
% correlación_trama
%------------------------------------------------- ------------------------

función plot_correlation(P)

ncol=256 ;

% mapa de colores frío caliente
cmap=hsv2rgb([linspace(2/3, 0, ncol)' 0.9*ones(ncol,1) unos(ncol,1)]) ;
cmap(1,:)=[0 0 0] ;
mapa de colores(cmap);

corr = correlación(P);
imágenesc(abs(corr), [0 1]); % sin señal

imagen del eje;
barra de colores;

fin

%------------------------------------------------- ------------------------
% correlación
%------------------------------------------------- ------------------------

función Corr=correlación(Cov)

sigmas = sqrt(diag(Cov));
Corr = diag(1./sigmas)*Cov*diag(1./sigmas);

fin
