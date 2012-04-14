sDataPath = '~/builds/build_fpl/Data/';

close all

std_noise  = 2; %noise in the measurements (in pixels)
zmin       = 2;

zmax_list=[2,4,8,12,20,30,40,50,60,70,80,90,100]

min_type = {'LU','LU-DIST','LU-ROB','LU-DIST-ROB'};

zmax_avg_errors = zeros(length(min_type),length(zmax_list));
zmax_med_errors = zeros(length(min_type),length(zmax_list));

opt_id = 1;
for opt_type=min_type
    zmax_id = 1;
    for zmax=zmax_list
        err = ...
            load( sprintf( [ sDataPath '/depth_%s_z%04d_z%04d_n%1.3f.txt' ], ...
                           opt_type{1}, zmin, zmax, std_noise ) );

        zmax_avg_errors(opt_id,zmax_id) = nanmean( err );
        zmax_med_errors(opt_id,zmax_id) = nanmedian( err );
        
        zmax_id = zmax_id+1;
    end
    opt_id = opt_id+1;
end

type_list = {'ro-','kd-','b.-','gx-','m--','bd-','b--','y+-','c.-'};

figure; hold on;
for ii=1:length(min_type)
  plot( zmax_list, zmax_avg_errors(ii,:), type_list{ii} );
end
h = legend( min_type, 'Location','NorthWest' );
set( h,'FontSize',16,'Interpreter','none')
xlabel( 'Depth ratio','FontSize',16); 
ylabel('Translation error','FontSize',16);
set(gca,'FontSize',16);

return

print( '-depsc2', 'avg_error_scene_factor.eps' )

figure; hold on;
for ii=1:length(min_type)
  plot( zmax_list, zmax_med_errors(ii,:), type_list{ii},'MarkerSize', ...
        8 );
end
h = legend( new_min_type, 'Location','NorthWest' );
set( h,'FontSize',16,'Interpreter','none')
xlabel( 'Depth ratio','FontSize',16); 
ylabel('Translation error','FontSize',16);
set(gca,'FontSize',16);
print( '-depsc2', 'med_error_scene_factor.eps' )