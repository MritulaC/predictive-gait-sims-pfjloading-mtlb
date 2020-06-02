% inputs

staticTRC = 'static.trc';
markerLabels = [{'RASI'}; {'LASI'}; {'RPSI'}; {'LPSI'}];
outputTRC = 'static_HJCadded.trc';

% Use the Vec3 TimeSeriesTable to read the Vec3 type data file.
staticTable = TimeSeriesTableVec3(staticTRC);
% Use the OpenSim Utility function, osimTable2Struct to
% convert the OpenSim table into a Matlab Struct for ease of use.
staticMarkerData = osimTableToStruct(staticTable);

%Get ASIS and PSIS data
%Also convert back to Vicon system and mm
RASIS = [staticMarkerData.(markerLabels{1})(:,3)*1000,...
    staticMarkerData.(markerLabels{1})(:,1)*1000,...
    staticMarkerData.(markerLabels{1})(:,2)*1000]';
LASIS = [staticMarkerData.(markerLabels{2})(:,3)*1000,...
    staticMarkerData.(markerLabels{2})(:,1)*1000,...
    staticMarkerData.(markerLabels{2})(:,2)*1000]';
RPSIS = [staticMarkerData.(markerLabels{3})(:,3)*1000,...
    staticMarkerData.(markerLabels{3})(:,1)*1000,...
    staticMarkerData.(markerLabels{3})(:,2)*1000]';
LPSIS = [staticMarkerData.(markerLabels{4})(:,3)*1000,...
    staticMarkerData.(markerLabels{4})(:,1)*1000,...
    staticMarkerData.(markerLabels{4})(:,2)*1000]';

%% Run calculations for hip joint centres

% % % calculations seem to be working now -- also should add in some
% % % virtual markers for knee joint centre, ankle joint centre and floor
% % % markers underneath the feet markers

% % % %scaling can work as
% % % femur = HJC to KJC
% % % shank = KJC to AJC
% % % feet body height = ajc to floor AJC
% % % feet length = ajc floor to mid MT floor
% % % feet width = MT1 floor to MT5 floor

for t=1:size(RASIS,2)

    %Right-handed Pelvis reference system definition 
    SACRUM(:,t)=(RPSIS(:,t)+LPSIS(:,t))/2;      
    %Global Pelvis Center position
    OP(:,t)=(LASIS(:,t)+RASIS(:,t))/2;    
    
    PROVV(:,t)=(RASIS(:,t)-SACRUM(:,t))/norm(RASIS(:,t)-SACRUM(:,t));  
    IB(:,t)=(RASIS(:,t)-LASIS(:,t))/norm(RASIS(:,t)-LASIS(:,t));    
    
    KB(:,t)=cross(IB(:,t),PROVV(:,t));                               
    KB(:,t)=KB(:,t)/norm(KB(:,t));
    
    JB(:,t)=cross(KB(:,t),IB(:,t));                               
    JB(:,t)=JB(:,t)/norm(JB(:,t));
    
    OB(:,t)=OP(:,t);
      
    %rotation+ traslation in homogeneous coordinates (4x4)
    pelvis(:,:,t)=[IB(:,t) JB(:,t) KB(:,t) OB(:,t);
                   0 0 0 1];
    
    %Trasformation into pelvis coordinate system (CS)
    OPB(:,t)=inv(pelvis(:,:,t))*[OB(:,t);1];    
       
    PW(t)=norm(RASIS(:,t)-LASIS(:,t));
    PD(t)=norm(SACRUM(:,t)-OP(:,t));
    
    %Harrington formulae (starting from pelvis center)
    diff_ap(t)=-0.24*PD(t)-9.9;
    diff_v(t)=-0.30*PW(t)-10.9;
    diff_ml(t)=0.33*PW(t)+7.3;
    
    %vector that must be subtract to OP to obtain hjc in pelvis CS
    %vett_diff_pelvis_sx(:,t)=[-diff_ml(t);diff_v(t);diff_ap(t);1];
    %vett_diff_pelvis_dx(:,t)=[diff_ml(t);diff_v(t);diff_ap(t);1];
    vett_diff_pelvis_sx(:,t)=[-diff_ml(t);diff_ap(t);diff_v(t);1];
    vett_diff_pelvis_dx(:,t)=[diff_ml(t);diff_ap(t);diff_v(t);1];    
    
    %hjc in pelvis CS (4x4)
    rhjc_pelvis(:,t)=OPB(:,t)+vett_diff_pelvis_dx(:,t);  
    lhjc_pelvis(:,t)=OPB(:,t)+vett_diff_pelvis_sx(:,t);  
    

    %Transformation Local to Global
    RHJC(:,t)=pelvis(1:3,1:3,t)*[rhjc_pelvis(1:3,t)]+OB(:,t);
    LHJC(:,t)=pelvis(1:3,1:3,t)*[lhjc_pelvis(1:3,t)]+OB(:,t);
    %or
    %RHJC(:,t)=pelvis(:,:,t)*[rhjc_pelvis(:,t)];
    %LHJC(:,t)=pelvis(:,:,t)*[lhjc_pelvis(:,t)];
    
    %Or other way to check the result:
    %transformation of the difference vector into global 
    %vett_diff_global_sx(:,t)=pelvis(:,:,t)*vett_diff_pelvis_sx(:,t);
    %vett_diff_global_dx(:,t)=pelvis(:,:,t)*vett_diff_pelvis_dx(:,t);
    %Sum global diff vect + global center
    %RHJC(:,t)=[OB(:,t);1]+vett_diff_global_dx(:,t);
    %LHJC(:,t)=[OB(:,t);1]+vett_diff_global_sx(:,t);
       
end

% To have a mean value during the whole static acquisition instead of for
% each time instant, mean computation must be added
% RHJC=mean(RHJC(:,:),2);
% LHJC=mean(LHJC(:,:),2);

% [time x [x y z]]
RHJC = RHJC';
LHJC = LHJC';

%Add HJC markers into matlab struct
%Convert back to opensim coordinate system and m
staticMarkerData.RHJC = [RHJC(:,2)/1000,RHJC(:,3)/1000,RHJC(:,1)/1000];
staticMarkerData.LHJC = [LHJC(:,2)/1000,LHJC(:,3)/1000,LHJC(:,1)/1000];

%% Export to new trc file

%Convert updated matlab struct back to osim table
newMarkerTable = osimTableFromStruct(staticMarkerData);

%Get the table meta data from the original table and add back into the new
%one

%Get meta data keys
metaKeys = staticTable.getTableMetaDataKeys();

%Loop through keys and append
for ii = 0:metaKeys.size()-1
    %Get current meta data string
    currMeta = char(staticTable.getTableMetaDataKeys().get(ii));
    %Append with relevant value
    newMarkerTable.addTableMetaDataString(currMeta,...
        staticTable.getTableMetaDataAsString(currMeta));
    %Cleanup
    clear currMeta
end

%Write to file
TRCFileAdapter().write(newMarkerTable,'new.trc')


