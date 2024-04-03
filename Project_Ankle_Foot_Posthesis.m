% Importing all OpenSim libraries.
import org.opensim.modeling.* 

% Instantiate an (empty) OpenSim Model.
model = Model();
model.setName('Project_Prosthetic_Foot');

% Get a reference to the ground object.
ground = model.getGround(); 

% Define the acceleration of gravity.
model.setGravity(Vec3(0,-9.80665,0)); 

% Bodies in the model.
Body1 = Body();
Body1.setName('Platform'); 
Body1.setMass(1);
Body1.setInertia( Inertia(1,1,1,0,0,0) ); 
Body1Geometry = Mesh('Prosthesis_Platform.stl'); 
Body1Geometry.set_scale_factors( Vec3(1,1,1) );
Body1Geometry.setColor( Vec3(0.7,0.4,0.2) );
Body1Geometry.setOpacity(1);
Body1.attachGeometry(Body1Geometry);
model.addBody(Body1);

Body2 = Body();
Body2.setName('FootBase'); 
Body2.setMass(0.585); 
Body2.setMassCenter( Vec3(0.04,-0.0335,0.0057) );
Body2.setInertia( Inertia(0,0.0035,0.00057,0,0,0) );
Body2Geometry1 = Mesh('Prosthesis_Base1.stl');  
Body2Geometry1.set_scale_factors( Vec3(1,1,1) ); 
Body2Geometry1.setColor( Vec3(0.1,0.1,0.1) ); 
Body2Geometry2 = Mesh('Prosthesis_Base2.stl');  
Body2Geometry2.set_scale_factors( Vec3(1,1,1) ); 
Body2Geometry2.setColor( Vec3(0.4,0.4,0.4) );
Body2.attachGeometry(Body2Geometry1);
Body2.attachGeometry(Body2Geometry2);
model.addBody(Body2);

Body3 = Body();
Body3.setName('FootUpper');
Body3.setMass(0.18);
Body3.setMassCenter( Vec3(0,0,0) );
Body3.setInertia( Inertia(0.0012,1,0.02,0,0,0) );
Body3Geometry = Mesh('Prosthesis_Upper.stl'); 
Body3Geometry.set_scale_factors( Vec3(0.0012,0.0012,0.0012) );
Body3Geometry.setColor( Vec3(0.7,0.7,0.7) ); 
Body3.attachGeometry(Body3Geometry);
model.addBody(Body3);

% Joints and coordinates in the model| Attached the bodies to the model.
GroundToBody1 = 0;     
Body2ToBody1  = 0.00;
Body2Height   = 0.20;
Body1Height   = 0.047;
Body1Length   = 0.270;

Joint1 = PinJoint('JointPlatform',...            % Platform Joint Name.
                  ground,...                     % Parent Frame.
                  Vec3(0,GroundToBody1,0),...    % Translation in Parent Frame.
                  Vec3(0,0,0),...                % Orientation in Parent Frame.
                  Body1,...                      % Child Frame.
                  Vec3(0,0,0),...                % Translation in Child Frame.
                  Vec3(0,0,0));                  % Orientation in Child Frame.    
Body1_rz = Joint1.upd_coordinates(0);            % Coordinate rz
Body1_rz.setRange([deg2rad(-17), deg2rad(9.5)]); % Coordinate range
Body1_rz.setName('Platform_rz');                 % Coordinate Name.
Body1_rz.setDefaultValue(deg2rad(-1.8));         % −1,81° to −7,08° (initial angle).
Body1_rz.setDefaultSpeedValue(0);                % Default Speed in 0 m/s.
Body1_rz.setDefaultLocked(false);                % Platform rz joint mobile.
model.addJoint(Joint1);                          % Adding the Platform Joint to the model.

Joint2 = PlanarJoint('JointFootUpper', ... 
                     ground, ...
                     Body3);
Body3_rz = Joint2.upd_coordinates(0); % Foot Upper Coordinate rz.
Body3_rz.setRange([deg2rad(0),deg2rad(0)]);
Body3_rz.setName('Foot_Upper_rz');
Body3_rz.setDefaultValue(0);
Body3_rz.setDefaultSpeedValue(0);
Body3_rz.setDefaultLocked(true);
Body3_tx = Joint2.upd_coordinates(1); % Foot Upper Coordinate tx.
Body3_tx.setRange([-2,3]);
Body3_tx.setName('Foot_Upper_tx');
Body3_tx.setDefaultValue(0);
Body3_tx.setDefaultSpeedValue(0);
Body3_tx.setDefaultLocked(true);
Body3_ty = Joint2.upd_coordinates(2); % Foot Upper  Coordinate ty.
Body3_ty.setRange([0,1]);
Body3_ty.setName('Foot_Upper_ty');
Body3_ty.setDefaultValue(...
       GroundToBody1 + Body2ToBody1 + Body1Height + Body2Height - 0.2);
Body3_ty.setDefaultSpeedValue(0);
Body3_ty.setDefaultLocked(false);
model.addJoint(Joint2);

Joint3 = PinJoint('JointAnkle', ...
                  Body3, ...
                  Vec3(0,0,0),...
                  Vec3(0,0,0),...
                  Body2, ...
                  Vec3(0,0,0),...
                  Vec3(0,0,0));
Body2_rz = Joint3.upd_coordinates(0); % Ankle Coordinate rz.
Body2_rz.setRange([deg2rad(-17),deg2rad(9.5)]); 
Body2_rz.setName('Ankle');
Body2_rz.setDefaultValue(deg2rad(-1.8));
Body2_rz.setDefaultSpeedValue(0);
model.addJoint(Joint3);

% Contact Geometry in the Model.
PlatformContactGeometry = ContactMesh();
PlatformContactGeometry.setFilename('Prosthesis_Platform.stl');
PlatformContactGeometry.setName('PlatformContact');
PlatformContactGeometry.setLocation( Vec3(0,0,0));
PlatformContactGeometry.setOrientation( Vec3(0,0,0) );
PlatformContactGeometry.setFrame(Body1);
model.addContactGeometry(PlatformContactGeometry);

FootContactGeometry = ContactMesh();
FootContactGeometry.setFilename('Prosthesis_Base1.stl');
FootContactGeometry.setName('FootContact');
FootContactGeometry.setLocation( Vec3(0,0,0) );    % SimTK::Vec3 & location.
FootContactGeometry.setOrientation( Vec3(0,0,0) ); % SimTK::Vec3 & orientation.
FootContactGeometry.setFrame(Body2);               % PhysicalFrame & frame.
model.addContactGeometry(FootContactGeometry);     % std::string &  name.

% Contact Forces in the Model (Elastic Foundation Force)
EFF = ElasticFoundationForce();
EFF.setName('EFF_FootPlatform');
EFF.setStiffness(1e6);              % Hertz stiffness   in N/m^2 
EFF.setDissipation(2.0);            % dissipation       in s/m
EFF.setStaticFriction(0.8);         % Stribeck friction in N/A 
EFF.setDynamicFriction(0.8);        % DynamicFriction   in N/A 
EFF.setViscousFriction(0.8);        % Viscous Friction  in N/A
EFF.addGeometry('PlatformContact');
EFF.addGeometry('FootContact');
model.addForce(EFF);

% Initialize the System
model.initSystem();
model.print('Project_Ankle.osim');   % command to save the file.osim.
disp('Project_Ankle.osim printed!'); % print at Command Window.
