/*!
 * To temelji na https://github.com/griffpatch/scratch-vm/tree/box2d/src/extensions/scratch3_griffpatch
 */

(funkcija(Scratch) {
  'use strict';

  if (!Scratch.extensions.unsandboxed) {
    throw new Error('Box2D je treba zagnati brez peskovnika');
  }

  // Najprej moramo naložiti fizikalno knjižnico Box2D, ki jo uporablja ta razširitev.
  // Trenutno ni priporočljivo nalagati več skriptov v razširitvah po meri, zato smo kopirali in prilepili
  // izvorna koda spodaj. Ja, to je res grdo.

   /*!
    * Avtorske pravice (c) 2006-2007 Erin Catto http://www.gphysics.com
    *
    * Ta programska oprema je na voljo "takšna, kot je", brez kakršnega koli izrecnega ali implicitnega
    * garancija. V nobenem primeru avtorji ne odgovarjajo za morebitno škodo
    * ki izhajajo iz uporabe te programske opreme.
    * Vsakemu je dovoljeno uporabljati to programsko opremo za kateri koli namen,
    * vključno s komercialnimi aplikacijami ter za njegovo spreminjanje in redistribucijo
    * prosto, ob upoštevanju naslednjih omejitev:
    * 1. Izvor te programske opreme se ne sme napačno predstavljati; ne smeš
    * trdite, da ste napisali izvirno programsko opremo. Če uporabljate to programsko opremo
    * v izdelku bi bila potrditev v dokumentaciji izdelka
    * cenjeno, vendar ni obvezno.
    * 2. Spremenjene izvorne različice morajo biti kot take jasno označene in ne smejo biti
    * napačno predstavljeno kot izvirna programska oprema.
    * 3. Tega obvestila ni dovoljeno odstraniti ali spremeniti iz katere koli izvorne distribucije.
    */

  var Box2D = {};
  (funkcija (a2j, nedefinirano) {
    funkcija emptyFn() {}
    a2j.inherit = funkcija (cls, osnova) {
      var tmpCtr = cls;
      prazniFn.prototip = osnova.prototip;
      cls.prototype = new emptyFn();
      cls.prototype.constructor = tmpCtr;
    };

    a2j.generateCallback = funkcija generateCallback(kontekst, cb) {
      funkcija vrnitve () {
        cb.apply(kontekst, argumenti);
      };
    };

    a2j.NVector = funkcija NVector(dolžina) {
      if (dolžina === nedefinirano) length = 0;
      var tmp = nova matrika (dolžina || 0);
      za (var i = 0; i < dolžina; ++i) tmp[i] = 0;
      vrni tmp;
    };

    a2j.is = funkcija je (o1, o2) {
      if (o1 === null) vrni false;
      if (o2 primerek funkcije && o1 primerek o2) vrne true;
      če (
        o1.constructor.__implements != nedefinirano &&
        o1.constructor.__implements[o2]
      )
        vrni resnico;
      vrni false;
    };

    a2j.parseUInt = funkcija (v) {
      vrni Math.abs(parseInt(v));
    };
  })(Polje2D);

  //#TODO odstranite dodelitve iz globalnega imenskega prostora
  var Vector = Array;
  var Vector_a2j_Number = Box2D.NVector;
  //struktura paketa
  if (typeof Box2D === "nedefinirano") Box2D = {};
  if (typeof Box2D.Collision === "nedefinirano") Box2D.Collision = {};
  if (typeof Box2D.Collision.Shapes === "nedefinirano") Box2D.Collision.Shapes = {};
  if (typeof Box2D.Common === "nedefinirano") Box2D.Common = {};
  if (typeof Box2D.Common.Math === "nedefinirano") Box2D.Common.Math = {};
  if (typeof Box2D.Dynamics === "nedefinirano") Box2D.Dynamics = {};
  if (typeof Box2D.Dynamics.Contacts === "nedefinirano")
    Box2D.Dynamics.Contacts = {};
  if (typeof Box2D.Dynamics.Controllers === "nedefinirano")
    Box2D.Dynamics.Controllers = {};
  if (typeof Box2D.Dynamics.Joints === "nedefinirano") Box2D.Dynamics.Joints = {};
  //preddefinicije
  (funkcija () {
    Box2D.Collision.IBroadPhase = "Box2D.Collision.IBroadPhase";

    funkcija b2AABB() {
      b2AABB.b2AABB.apply(to, argumenti);
    }
    Box2D.Collision.b2AABB = b2AABB;

    funkcija b2Bound() {
      b2Bound.b2Bound.apply(to, argumenti);
    }
    Box2D.Collision.b2Bound = b2Bound;

    funkcija b2BoundValues() {
      b2BoundValues.b2BoundValues.apply(to, argumenti);
      if (this.constructor === b2BoundValues)
        this.b2BoundValues.apply(this, argumenti);
    }
    Box2D.Collision.b2BoundValues ​​= b2BoundValues;

    funkcija b2Collision() {
      b2Collision.b2Collision.apply(to, argumenti);
    }
    Box2D.Collision.b2Collision = b2Collision;

    funkcija b2ContactID() {
      b2ContactID.b2ContactID.apply(to, argumenti);
      če (this.constructor === b2ContactID)
        this.b2ContactID.apply(to, argumenti);
    }
    Box2D.Collision.b2ContactID = b2ContactID;

    funkcija b2ContactPoint() {
      b2ContactPoint.b2ContactPoint.apply(to, argumenti);
    }
    Box2D.Collision.b2ContactPoint = b2ContactPoint;

    funkcija b2Distance() {
      b2Distance.b2Distance.apply(to, argumenti);
    }
    Box2D.Collision.b2Distance = b2Distance;

    funkcija b2DistanceInput() {
      b2DistanceInput.b2DistanceInput.apply(to, argumenti);
    }
    Box2D.Collision.b2DistanceInput = b2DistanceInput;

    funkcija b2DistanceOutput() {
      b2DistanceOutput.b2DistanceOutput.apply(to, argumenti);
    }
    Box2D.Collision.b2DistanceOutput = b2DistanceOutput;

    funkcija b2DistanceProxy() {
      b2DistanceProxy.b2DistanceProxy.apply(to, argumenti);
    }
    Box2D.Collision.b2DistanceProxy = b2DistanceProxy;

    funkcija b2DynamicTree() {
      b2DynamicTree.b2DynamicTree.apply(to, argumenti);
      če (this.constructor === b2DynamicTree)
        this.b2DynamicTree.apply(this, argumenti);
    }
    Box2D.Collision.b2DynamicTree = b2DynamicTree;

    funkcija b2DynamicTreeBroadPhase() {
      b2DynamicTreeBroadPhase.b2DynamicTreeBroadPhase.apply(to, argumenti);
    }
    Box2D.Collision.b2DynamicTreeBroadPhase = b2DynamicTreeBroadPhase;

    funkcija b2DynamicTreeNode() {
      b2DynamicTreeNode.b2DynamicTreeNode.apply(to, argumenti);
    }
    Box2D.Collision.b2DynamicTreeNode = b2DynamicTreeNode;

    funkcija b2DynamicTreePair() {
      b2DynamicTreePair.b2DynamicTreePair.apply(to, argumenti);
    }
    Box2D.Collision.b2DynamicTreePair = b2DynamicTreePair;

    funkcija b2Manifold() {
      b2Manifold.b2Manifold.apply(to, argumenti);
      if (this.constructor === b2Manifold) this.b2Manifold.apply(this, arguments);
    }
    Box2D.Collision.b2Manifold = b2Manifold;

    funkcija b2ManifoldPoint() {
      b2ManifoldPoint.b2ManifoldPoint.apply(to, argumenti);
      če (this.constructor === b2ManifoldPoint)
        this.b2ManifoldPoint.apply(this, argumenti);
    }
    Box2D.Collision.b2ManifoldPoint = b2ManifoldPoint;

    funkcija b2Point() {
      b2Point.b2Point.apply(to, argumenti);
    }
    Box2D.Collision.b2Point = b2Point;

    funkcija b2RayCastInput() {
      b2RayCastInput.b2RayCastInput.apply(to, argumenti);
      če (this.constructor === b2RayCastInput)
        this.b2RayCastInput.apply(this, argumenti);
    }
    Box2D.Collision.b2RayCastInput = b2RayCastInput;

    funkcija b2RayCastOutput() {
      b2RayCastOutput.b2RayCastOutput.apply(to, argumenti);
    }
    Box2D.Collision.b2RayCastOutput = b2RayCastOutput;

    funkcija b2Segment() {
      b2Segment.b2Segment.apply(to, argumenti);
    }
    Box2D.Collision.b2Segment = b2Segment;

    funkcija b2SeparationFunction() {
      b2SeparationFunction.b2SeparationFunction.apply(to, argumenti);
    }
    Box2D.Collision.b2SeparationFunction = b2SeparationFunction;

    funkcija b2Simplex() {
      b2Simplex.b2Simplex.apply(to, argumenti);
      if (this.constructor === b2Simplex) this.b2Simplex.apply(this, arguments);
    }
    Box2D.Collision.b2Simplex = b2Simplex;

    funkcija b2SimplexCache() {
      b2SimplexCache.b2SimplexCache.apply(to, argumenti);
    }
    Box2D.Collision.b2SimplexCache = b2SimplexCache;

    funkcija b2SimplexVertex() {
      b2SimplexVertex.b2SimplexVertex.apply(to, argumenti);
    }
    Box2D.Collision.b2SimplexVertex = b2SimplexVertex;

    funkcija b2TimeOfImpact() {
      b2TimeOfImpact.b2TimeOfImpact.apply(to, argumenti);
    }
    Box2D.Collision.b2TimeOfImpact = b2TimeOfImpact;

    funkcija b2TOIInput() {
      b2TOIInput.b2TOIInput.apply(to, argumenti);
    }
    Box2D.Collision.b2TOIInput = b2TOIInput;

    funkcija b2WorldManifold() {
      b2WorldManifold.b2WorldManifold.apply(to, argumenti);
      če (this.constructor === b2WorldManifold)
        this.b2WorldManifold.apply(to, argumenti);
    }
    Box2D.Collision.b2WorldManifold = b2WorldManifold;

    funkcija ClipVertex() {
      ClipVertex.ClipVertex.apply(to, argumenti);
    }
    Box2D.Collision.ClipVertex = ClipVertex;

    funkcija Features() {
      Features.Features.apply(this, arguments);
    }
    Box2D.Collision.Features = Lastnosti;

    funkcija b2CircleShape() {
      b2CircleShape.b2CircleShape.apply(to, argumenti);
      če (this.constructor === b2CircleShape)
        this.b2CircleShape.apply(this, argumenti);
    }
    Box2D.Collision.Shapes.b2CircleShape = b2CircleShape;

    funkcija b2EdgeChainDef() {
      b2EdgeChainDef.b2EdgeChainDef.apply(to, argumenti);
      če (this.constructor === b2EdgeChainDef)
        this.b2EdgeChainDef.apply(this, argumenti);
    }
    Box2D.Collision.Shapes.b2EdgeChainDef = b2EdgeChainDef;

    funkcija b2EdgeShape() {
      b2EdgeShape.b2EdgeShape.apply(to, argumenti);
      če (this.constructor === b2EdgeShape)
        this.b2EdgeShape.apply(this, argumenti);
    }
    Box2D.Collision.Shapes.b2EdgeShape = b2EdgeShape;

    funkcija b2MassData() {
      b2MassData.b2MassData.apply(to, argumenti);
    }
    Box2D.Collision.Shapes.b2MassData = b2MassData;

    funkcija b2PolygonShape() {
      b2PolygonShape.b2PolygonShape.apply(to, argumenti);
      če (this.constructor === b2PolygonShape)
        this.b2PolygonShape.apply(this, argumenti);
    }
    Box2D.Collision.Shapes.b2PolygonShape = b2PolygonShape;

    funkcija b2Shape() {
      b2Shape.b2Shape.apply(to, argumenti);
      if (this.constructor === b2Shape) this.b2Shape.apply(this, arguments);
    }
    Box2D.Collision.Shapes.b2Shape = b2Shape;
    Box2D.Common.b2internal = "Box2D.Common.b2internal";

    funkcija b2Color() {
      b2Color.b2Color.apply(to, argumenti);
      if (this.constructor === b2Color) this.b2Color.apply(this, argumenti);
    }
    Box2D.Common.b2Color = b2Color;

    funkcija b2Settings() {
      b2Settings.b2Settings.apply(to, argumenti);
    }
    Box2D.Common.b2Settings = b2Settings;

    funkcija b2Mat22() {
      b2Mat22.b2Mat22.apply(to, argumenti);
      if (this.constructor === b2Mat22) this.b2Mat22.apply(this, argumenti);
    }
    Box2D.Common.Math.b2Mat22 = b2Mat22;

    funkcija b2Mat33() {
      b2Mat33.b2Mat33.apply(to, argumenti);
      if (this.constructor === b2Mat33) this.b2Mat33.apply(this, arguments);
    }
    Box2D.Common.Math.b2Mat33 = b2Mat33;

    funkcija b2Math() {
      b2Math.b2Math.apply(to, argumenti);
    }
    Box2D.Common.Math.b2Math = b2Math;

    funkcija b2Sweep() {
      b2Sweep.b2Sweep.apply(to, argumenti);
    }
    Box2D.Common.Math.b2Sweep = b2Sweep;

    funkcija b2Transform() {
      b2Transform.b2Transform.apply(to, argumenti);
      če (this.constructor === b2Transform)
        this.b2Transform.apply(this, argumenti);
    }
    Box2D.Common.Math.b2Transform = b2Transform;

    funkcija b2Vec2() {
      b2Vec2.b2Vec2.apply(to, argumenti);
      if (this.constructor === b2Vec2) this.b2Vec2.apply(this, argumenti);
    }
    Box2D.Common.Math.b2Vec2 = b2Vec2;

    funkcija b2Vec3() {
      b2Vec3.b2Vec3.apply(to, argumenti);
      if (this.constructor === b2Vec3) this.b2Vec3.apply(this, argumenti);
    }
    Box2D.Common.Math.b2Vec3 = b2Vec3;

    funkcija b2Body() {
      b2Body.b2Body.apply(to, argumenti);
      if (this.constructor === b2Body) this.b2Body.apply(this, arguments);
    }
    Box2D.Dynamics.b2Body = b2Body;

    funkcija b2BodyDef() {
      b2BodyDef.b2BodyDef.apply(to, argumenti);
      if (this.constructor === b2BodyDef) this.b2BodyDef.apply(this, argumenti);
    }
    Box2D.Dynamics.b2BodyDef = b2BodyDef;

    funkcija b2ContactFilter() {
      b2ContactFilter.b2ContactFilter.apply(to, argumenti);
    }
    Box2D.Dynamics.b2ContactFilter = b2ContactFilter;

    funkcija b2ContactImpulse() {
      b2ContactImpulse.b2ContactImpulse.apply(to, argumenti);
    }
    Box2D.Dynamics.b2ContactImpulse = b2ContactImpulse;

    funkcija b2ContactListener() {
      b2ContactListener.b2ContactListener.apply(to, argumenti);
    }
    Box2D.Dynamics.b2ContactListener = b2ContactListener;

    funkcija b2ContactManager() {
      b2ContactManager.b2ContactManager.apply(to, argumenti);
      če (this.constructor === b2ContactManager)
        this.b2ContactManager.apply(to, argumenti);
    }
    Box2D.Dynamics.b2ContactManager = b2ContactManager;

    funkcija b2DebugDraw() {
      b2DebugDraw.b2DebugDraw.apply(to, argumenti);
      če (this.constructor === b2DebugDraw)
        this.b2DebugDraw.apply(this, argumenti);
    }
    Box2D.Dynamics.b2DebugDraw = b2DebugDraw;

    funkcija b2DestructionListener() {
      b2DestructionListener.b2DestructionListener.apply(to, argumenti);
    }
    Box2D.Dynamics.b2DestructionListener = b2DestructionListener;

    funkcija b2FilterData() {
      b2FilterData.b2FilterData.apply(to, argumenti);
    }
    Box2D.Dynamics.b2FilterData = b2FilterData;

    funkcija b2Fixture() {
      b2Fixture.b2Fixture.apply(to, argumenti);
      if (this.constructor === b2Fixture) this.b2Fixture.apply(this, arguments);
    }
    Box2D.Dynamics.b2Fixture = b2Fixture;

    funkcija b2FixtureDef() {
      b2FixtureDef.b2FixtureDef.apply(to, argumenti);
      če (this.constructor === b2FixtureDef)
        this.b2FixtureDef.apply(this, argumenti);
    }
    Box2D.Dynamics.b2FixtureDef = b2FixtureDef;

    funkcija b2Island() {
      b2Island.b2Island.apply(to, argumenti);
      if (this.constructor === b2Island) this.b2Island.apply(this, arguments);
    }
    Box2D.Dynamics.b2Island = b2Island;

    funkcija b2TimeStep() {
      b2TimeStep.b2TimeStep.apply(to, argumenti);
    }
    Box2D.Dynamics.b2TimeStep = b2TimeStep;

    funkcija b2World() {
      b2World.b2World.apply(to, argumenti);
      if (this.constructor === b2World) this.b2World.apply(this, arguments);
    }
    Box2D.Dynamics.b2World = b2World;

    funkcija b2CircleContact() {
      b2CircleContact.b2CircleContact.apply(to, argumenti);
    }
    Box2D.Dynamics.Contacts.b2CircleContact = b2CircleContact;

    funkcija b2Contact() {
      b2Contact.b2Contact.apply(to, argumenti);
      if (this.constructor === b2Contact) this.b2Contact.apply(this, arguments);
    }
    Box2D.Dynamics.Contacts.b2Contact = b2Contact;

    funkcija b2ContactConstraint() {
      b2ContactConstraint.b2ContactConstraint.apply(to, argumenti);
      če (this.constructor === b2ContactConstraint)
        this.b2ContactConstraint.apply(this, argumenti);
    }
    Box2D.Dynamics.Contacts.b2ContactConstraint = b2ContactConstraint;

    funkcija b2ContactConstraintPoint() {
      b2ContactConstraintPoint.b2ContactConstraintPoint.apply(to, argumenti);
    }
    Box2D.Dynamics.Contacts.b2ContactConstraintPoint = b2ContactConstraintPoint;

    funkcija b2ContactEdge() {
      b2ContactEdge.b2ContactEdge.apply(to, argumenti);
    }
    Box2D.Dynamics.Contacts.b2ContactEdge = b2ContactEdge;

    funkcija b2ContactFactory() {
      b2ContactFactory.b2ContactFactory.apply(to, argumenti);
      če (this.constructor === b2ContactFactory)
        this.b2ContactFactory.apply(this, argumenti);
    }
    Box2D.Dynamics.Contacts.b2ContactFactory = b2ContactFactory;

    funkcija b2ContactRegister() {
      b2ContactRegister.b2ContactRegister.apply(to, argumenti);
    }
    Box2D.Dynamics.Contacts.b2ContactRegister = b2ContactRegister;

    funkcija b2ContactResult() {
      b2ContactResult.b2ContactResult.apply(to, argumenti);
    }
    Box2D.Dynamics.Contacts.b2ContactResult = b2ContactResult;

    funkcija b2ContactSolver() {
      b2ContactSolver.b2ContactSolver.apply(to, argumenti);
      če (this.constructor === b2ContactSolver)
        this.b2ContactSolver.apply(this, argumenti);
    }
    Box2D.Dynamics.Contacts.b2ContactSolver = b2ContactSolver;

    funkcija b2EdgeAndCircleContact() {
      b2EdgeAndCircleContact.b2EdgeAndCircleContact.apply(to, argumenti);
    }
    Box2D.Dynamics.Contacts.b2EdgeAndCircleContact = b2EdgeAndCircleContact;

    funkcija b2NullContact() {
      b2NullContact.b2NullContact.apply(to, argumenti);
      če (this.constructor === b2NullContact)
        this.b2NullContact.apply(this, argumenti);
    }
    Box2D.Dynamics.Contacts.b2NullContact = b2NullContact;

    funkcija b2PolyAndCircleContact() {
      b2PolyAndCircleContact.b2PolyAndCircleContact.apply(to, argumenti);
    }
    Box2D.Dynamics.Contacts.b2PolyAndCircleContact = b2PolyAndCircleContact;

    funkcija b2PolyAndEdgeContact() {
      b2PolyAndEdgeContact.b2PolyAndEdgeContact.apply(to, argumenti);
    }
    Box2D.Dynamics.Contacts.b2PolyAndEdgeContact = b2PolyAndEdgeContact;

    funkcija b2PolygonContact() {
      b2PolygonContact.b2PolygonContact.apply(to, argumenti);
    }
    Box2D.Dynamics.Contacts.b2PolygonContact = b2PolygonContact;

    funkcija b2PositionSolverManifold() {
      b2PositionSolverManifold.b2PositionSolverManifold.apply(to, argumenti);
      če (this.constructor === b2PositionSolverManifold)
        this.b2PositionSolverManifold.apply(this, argumenti);
    }
    Box2D.Dynamics.Contacts.b2PositionSolverManifold = b2PositionSolverManifold;

    funkcija b2BuoyancyController() {
      b2BuoyancyController.b2BuoyancyController.apply(to, argumenti);
    }
    Box2D.Dynamics.Controllers.b2BuoyancyController = b2BuoyancyController;

    funkcija b2ConstantAccelController() {
      b2ConstantAccelController.b2ConstantAccelController.apply(to, argumenti);
    }
    Box2D.Dynamics.Controllers.b2ConstantAccelController =
      b2ConstantAccelController;

    funkcija b2ConstantForceController() {
      b2ConstantForceController.b2ConstantForceController.apply(to, argumenti);
    }
    Box2D.Dynamics.Controllers.b2ConstantForceController =
      b2ConstantForceController;

    funkcija b2Controller() {
      b2Controller.b2Controller.apply(to, argumenti);
    }
    Box2D.Dynamics.Controllers.b2Controller = b2Controller;

    funkcija b2ControllerEdge() {
      b2ControllerEdge.b2ControllerEdge.apply(to, argumenti);
    }
    Box2D.Dynamics.Controllers.b2ControllerEdge = b2ControllerEdge;

    funkcija b2GravityController() {
      b2GravityController.b2GravityController.apply(to, argumenti);
    }
    Box2D.Dynamics.Controllers.b2GravityController = b2GravityController;

    funkcija b2TensorDampingController() {
      b2TensorDampingController.b2TensorDampingController.apply(to, argumenti);
    }
    Box2D.Dynamics.Controllers.b2TensorDampingController =
      b2TensorDampingController;

    funkcija b2DistanceJoint() {
      b2DistanceJoint.b2DistanceJoint.apply(to, argumenti);
      če (this.constructor === b2DistanceJoint)
        this.b2DistanceJoint.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2DistanceJoint = b2DistanceJoint;

    funkcija b2DistanceJointDef() {
      b2DistanceJointDef.b2DistanceJointDef.apply(to, argumenti);
      če (this.constructor === b2DistanceJointDef)
        this.b2DistanceJointDef.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2DistanceJointDef = b2DistanceJointDef;

    funkcija b2FrictionJoint() {
      b2FrictionJoint.b2FrictionJoint.apply(to, argumenti);
      če (this.constructor === b2FrictionJoint)
        this.b2FrictionJoint.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2FrictionJoint = b2FrictionJoint;

    funkcija b2FrictionJointDef() {
      b2FrictionJointDef.b2FrictionJointDef.apply(to, argumenti);
      če (this.constructor === b2FrictionJointDef)
        this.b2FrictionJointDef.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2FrictionJointDef = b2FrictionJointDef;

    funkcija b2GearJoint() {
      b2GearJoint.b2GearJoint.apply(to, argumenti);
      če (this.constructor === b2GearJoint)
        this.b2GearJoint.apply(to, argumenti);
    }
    Box2D.Dynamics.Joints.b2GearJoint = b2GearJoint;

    funkcija b2GearJointDef() {
      b2GearJointDef.b2GearJointDef.apply(to, argumenti);
      če (this.constructor === b2GearJointDef)
        this.b2GearJointDef.apply(to, argumenti);
    }
    Box2D.Dynamics.Joints.b2GearJointDef = b2GearJointDef;

    funkcija b2Jacobian() {
      b2Jacobian.b2Jacobian.apply(to, argumenti);
    }
    Box2D.Dynamics.Joints.b2Jacobian = b2Jacobian;

    funkcija b2Joint() {
      b2Joint.b2Joint.apply(to, argumenti);
      if (this.constructor === b2Joint) this.b2Joint.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2Joint = b2Joint;

    funkcija b2JointDef() {
      b2JointDef.b2JointDef.apply(to, argumenti);
      if (this.constructor === b2JointDef) this.b2JointDef.apply(this, arguments);
    }
    Box2D.Dynamics.Joints.b2JointDef = b2JointDef;

    funkcija b2JointEdge() {
      b2JointEdge.b2JointEdge.apply(to, argumenti);
    }
    Box2D.Dynamics.Joints.b2JointEdge = b2JointEdge;

    funkcija b2LineJoint() {
      b2LineJoint.b2LineJoint.apply(to, argumenti);
      če (this.constructor === b2LineJoint)
        this.b2LineJoint.apply(to, argumenti);
    }
    Box2D.Dynamics.Joints.b2LineJoint = b2LineJoint;

    funkcija b2LineJointDef() {
      b2LineJointDef.b2LineJointDef.apply(to, argumenti);
      če (this.constructor === b2LineJointDef)
        this.b2LineJointDef.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2LineJointDef = b2LineJointDef;

    funkcija b2MouseJoint() {
      b2MouseJoint.b2MouseJoint.apply(to, argumenti);
      če (this.constructor === b2MouseJoint)
        this.b2MouseJoint.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2MouseJoint = b2MouseJoint;

    funkcija b2MouseJointDef() {
      b2MouseJointDef.b2MouseJointDef.apply(to, argumenti);
      če (this.constructor === b2MouseJointDef)
        this.b2MouseJointDef.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2MouseJointDef = b2MouseJointDef;

    funkcija b2PrismaticJoint() {
      b2PrismaticJoint.b2PrismaticJoint.apply(to, argumenti);
      če (this.constructor === b2PrismaticJoint)
        this.b2PrismaticJoint.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2PrismaticJoint = b2PrismaticJoint;

    funkcija b2PrismaticJointDef() {
      b2PrismaticJointDef.b2PrismaticJointDef.apply(to, argumenti);
      če (this.constructor === b2PrismaticJointDef)
        this.b2PrismaticJointDef.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2PrismaticJointDef = b2PrismaticJointDef;

    funkcija b2PulleyJoint() {
      b2PulleyJoint.b2PulleyJoint.apply(to, argumenti);
      če (this.constructor === b2PulleyJoint)
        this.b2PulleyJoint.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2PulleyJoint = b2PulleyJoint;

    funkcija b2PulleyJointDef() {
      b2PulleyJointDef.b2PulleyJointDef.apply(to, argumenti);
      če (this.constructor === b2PulleyJointDef)
        this.b2PulleyJointDef.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2PulleyJointDef = b2PulleyJointDef;

    funkcija b2RevoluteJoint() {
      b2RevoluteJoint.b2RevoluteJoint.apply(to, argumenti);
      če (this.constructor === b2RevoluteJoint)
        this.b2RevoluteJoint.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2RevoluteJoint = b2RevoluteJoint;

    funkcija b2RevoluteJointDef() {
      b2RevoluteJointDef.b2RevoluteJointDef.apply(to, argumenti);
      če (this.constructor === b2RevoluteJointDef)
        this.b2RevoluteJointDef.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2RevoluteJointDef = b2RevoluteJointDef;

    funkcija b2WeldJoint() {
      b2WeldJoint.b2WeldJoint.apply(to, argumenti);
      če (this.constructor === b2WeldJoint)
        this.b2WeldJoint.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2WeldJoint = b2WeldJoint;

    funkcija b2WeldJointDef() {
      b2WeldJointDef.b2WeldJointDef.apply(to, argumenti);
      če (this.constructor === b2WeldJointDef)
        this.b2WeldJointDef.apply(this, argumenti);
    }
    Box2D.Dynamics.Joints.b2WeldJointDef = b2WeldJointDef;
  })(); //definicije
  Box2D.postDefs = [];
  (funkcija () {
    var b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
      b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef,
      b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape,
      b2MassData = Box2D.Collision.Shapes.b2MassData,
      b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
      b2Shape = Box2D.Collision.Shapes.b2Shape,
      b2Color = Box2D.Common.b2Color,
      b2notranji = Box2D.Common.b2notranji,
      b2Settings = Box2D.Common.b2Settings,
      b2Mat22 = Box2D.Common.Math.b2Mat22,
      b2Mat33 = Box2D.Common.Math.b2Mat33,
      b2Math = Box2D.Common.Math.b2Math,
      b2Sweep = Box2D.Common.Math.b2Sweep,
      b2Transform = Box2D.Common.Math.b2Transform,
      b2Vec2 = Box2D.Common.Math.b2Vec2,
      b2Vec3 = Box2D.Common.Math.b2Vec3,
      b2AABB = Box2D.Collision.b2AABB,
      b2Bound = Box2D.Collision.b2Bound,
      b2BoundValues ​​= Box2D.Collision.b2BoundValues,
      b2Collision = Box2D.Collision.b2Collision,
      b2ContactID = Box2D.Collision.b2ContactID,
      b2ContactPoint = Box2D.Collision.b2ContactPoint,
      b2Distance = Box2D.Collision.b2Distance,
      b2DistanceInput = Box2D.Collision.b2DistanceInput,
      b2DistanceOutput = Box2D.Collision.b2DistanceOutput,
      b2DistanceProxy = Box2D.Collision.b2DistanceProxy,
      b2DynamicTree = Box2D.Collision.b2DynamicTree,
      b2DynamicTreeBroadPhase = Box2D.Collision.b2DynamicTreeBroadPhase,
      b2DynamicTreeNode = Box2D.Collision.b2DynamicTreeNode,
      b2DynamicTreePair = Box2D.Collision.b2DynamicTreePair,
      b2Manifold = Box2D.Collision.b2Manifold,
      b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint,
      b2Point = Box2D.Collision.b2Point,
      b2RayCastInput = Box2D.Collision.b2RayCastInput,
      b2RayCastOutput = Box2D.Collision.b2RayCastOutput,
      b2Segment = Box2D.Collision.b2Segment,
      b2SeparationFunction = Box2D.Collision.b2SeparationFunction,
      b2Simplex = Box2D.Collision.b2Simplex,
      b2SimplexCache = Box2D.Collision.b2SimplexCache,
      b2SimplexVertex = Box2D.Collision.b2SimplexVertex,
      b2TimeOfImpact = Box2D.Collision.b2TimeOfImpact,
      b2TOIInput = Box2D.Collision.b2TOIInput,
      b2WorldManifold = Box2D.Collision.b2WorldManifold,
      ClipVertex = Box2D.Collision.ClipVertex,
      Lastnosti = Box2D.Collision.Features,
      IBroadPhase = Box2D.Collision.IBroadPhase;

    b2AABB.b2AABB = funkcija () {
      this.lowerBound = novo b2Vec2();
      this.upperBound = novo b2Vec2();
    };
    b2AABB.prototype.IsValid = funkcija () {
      var dX = this.upperBound.x - this.lowerBound.x;
      var dY = this.upperBound.y - this.lowerBound.y;
      veljavna var = dX >= 0,0 && dY >= 0,0;
      veljavno = veljavno && this.lowerBound.IsValid() && this.upperBound.IsValid();
      vrnitev veljavna;
    };
    b2AABB.prototype.GetCenter = funkcija () {
      vrni novo b2Vec2(
        (ta.spodnja meja.x + ta.zgornja meja.x) / 2,
        (ta.spodnja meja.y + ta.zgornja meja.y) / 2
      );
    };
    b2AABB.prototype.GetExtents = funkcija () {
      vrni novo b2Vec2(
        (ta.zgornja meja.x - ta.spodnja meja.x) / 2,
        (ta.zgornja meja.y - ta.spodnja meja.y) / 2
      );
    };
    b2AABB.prototype.Contains = funkcija (aabb) {
      var rezultat = res;
      rezultat = rezultat && this.lowerBound.x <= aabb.lowerBound.x;
      rezultat = rezultat && this.lowerBound.y <= aabb.lowerBound.y;
      rezultat = rezultat && aabb.upperBound.x <= this.upperBound.x;
      rezultat = rezultat && aabb.upperBound.y <= this.upperBound.y;
      vrni rezultat;
    };
    b2AABB.prototype.RayCast = funkcija (izhod, vnos) {
      var tmin = -Number.MAX_VALUE;
      var tmax = Number.MAX_VALUE;
      var pX = input.p1.x;
      var pY = input.p1.y;
      var dX = input.p2.x - input.p1.x;
      var dY = input.p2.y - input.p1.y;
      var absDX = Math.abs(dX);
      var absDY = Math.abs(dY);
      var normal = output.normal;
      var inv_d = 0;
      var t1 = 0;
      var t2 = 0;
      var t3 = 0;
      var s = 0;
      {
        if (absDX < Number.MIN_VALUE) {
          if (pX < this.lowerBound.x || this.upperBound.x < pX) return false;
        } drugače {
          inv_d = 1,0 / dX;
          t1 = (this.lowerBound.x - pX) * inv_d;
          t2 = (ta.zgornja meja.x - pX) * inv_d;
          s = -1,0;
          if (t1 > t2) {
            t3 = t1;
            t1 = t2;
            t2 = t3;
            s = 1,0;
          }
          if (t1 > tmin) {
            normalno.x = s;
            normalno.y = 0;
            tmin = t1;
          }
          tmax = Math.min(tmax, t2);
          if (tmin > tmax) vrni false;
        }
      }
      {
        if (absDY < Number.MIN_VALUE) {
          if (pY < this.lowerBound.y || this.upperBound.y < pY) return false;
        } drugače {
          inv_d = 1,0 / dY;
          t1 = (ta.spodnja meja.y - pY) * inv_d;
          t2 = (ta.zgornja meja.y - pY) * inv_d;
          s = -1,0;
          if (t1 > t2) {
            t3 = t1;
            t1 = t2;
            t2 = t3;
            s = 1,0;
          }
          if (t1 > tmin) {
            normalno.y = s;
            normalno.x = 0;
            tmin = t1;
          }
          tmax = Math.min(tmax, t2);
          if (tmin > tmax) vrni false;
        }
      }
      output.fraction = tmin;
      vrni resnico;
    };
    b2AABB.prototype.TestOverlap = funkcija (drugo) {
      var d1X = other.lowerBound.x - this.upperBound.x;
      var d1Y = other.lowerBound.y - this.upperBound.y;
      var d2X = this.lowerBound.x - other.upperBound.x;
      var d2Y = this.lowerBound.y - other.upperBound.y;
      if (d1X > 0,0 || d1Y > 0,0) vrni false;
      if (d2X > 0,0 || d2Y > 0,0) vrni false;
      vrni resnico;
    };
    b2AABB.Combine = funkcija (aabb1, aabb2) {
      var aabb = novo b2AABB();
      aabb.Združi (aabb1, aabb2);
      vrnitev aabb;
    };
    b2AABB.prototype.Combine = funkcija (aabb1, aabb2) {
      this.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
      this.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
      this.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
      this.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y);
    };
    b2Bound.b2Bound = funkcija () {};
    b2Bound.prototype.IsLower = funkcija () {
      return (ta.vrednost & 1) == 0;
    };
    b2Bound.prototype.IsUpper = funkcija () {
      return (ta.vrednost & 1) == 1;
    };
    b2Bound.prototype.Swap = funkcija (b) {
      var tempValue = this.value;
      var tempProxy = this.proxy;
      var tempStabbingCount = this.stabbingCount;
      this.value = b.value;
      this.proxy = b.proxy;
      this.stabbingCount = b.stabbingCount;
      b.value = tempValue;
      b.proxy = tempProxy;
      b.stabbingCount = tempStabbingCount;
    };
    b2BoundValues.b2BoundValues ​​= funkcija () {};
    b2BoundValues.prototype.b2BoundValues ​​= funkcija () {
      this.lowerValues ​​= new Vector_a2j_Number();
      this.lowerValues[0] = 0,0;
      this.lowerValues[1] = 0,0;
      this.upperValues ​​= new Vector_a2j_Number();
      this.upperValues[0] = 0,0;
      this.upperValues[1] = 0,0;
    };
    b2Collision.b2Collision = funkcija () {};
    b2Collision.ClipSegmentToLine = funkcija (vOut, vIn, normal, offset) {
      če (odmik === nedefiniran) odmik = 0;
      var cv;
      var numOut = 0;
      cv = vIn[0];
      var vIn0 = cv.v;
      cv = vIn[1];
      var vIn1 = cv.v;
      var distance0 = normal.x * vIn0.x + normal.y * vIn0.y - odmik;
      var distance1 = normal.x * vIn1.x + normal.y * vIn1.y - odmik;
      if (razdalja0 <= 0,0) vOut[numOut++].Set(vIn[0]);
      if (razdalja1 <= 0,0) vOut[numOut++].Set(vIn[1]);
      če (razdalja0 * razdalja1 < 0,0) {
        var interp = razdalja0 / (razdalja0 - razdalja1);
        cv = vIzhod[številoIzhod];
        var tVec = cv.v;
        tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x);
        tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y);
        cv = vIzhod[številoIzhod];
        var cv2;
        če (razdalja0 > 0,0) {
          cv2 = vIn[0];
          cv.id = cv2.id;
        } drugače {
          cv2 = vIn[1];
          cv.id = cv2.id;
        }
        ++numOut;
      }
      vrni numOut;
    };
    b2Collision.EdgeSeparation = funkcija (poli1, xf1, rob1, poli2, xf2) {
      če (rob1 === nedefiniran) rob1 = 0;
      var count1 = parseInt(poly1.m_vertexCount);
      var vertices1 = poly1.m_vertices;
      var normals1 = poly1.m_normals;
      var count2 = parseInt(poly2.m_vertexCount);
      var vertices2 = poly2.m_vertices;
      var tMat;
      var tVec;
      tMat = xf1.R;
      tVec = normals1[edge1];
      var normal1WorldX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      var normal1WorldY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tMat = xf2.R;
      var normal1X = tMat.col1.x * normal1WorldX + tMat.col1.y * normal1WorldY;
      var normal1Y = tMat.col2.x * normal1WorldX + tMat.col2.y * normal1WorldY;
      spremenljiv indeks = 0;
      var minDot = Number.MAX_VALUE;
      for (var i = 0; i < count2; ++i) {
        tVec = vozlišča2[i];
        var dot = tVec.x * normal1X + tVec.y * normal1Y;
        if (dot < minDot) {
          minDot = pika;
          indeks = i;
        }
      }
      tVec = vozlišča1[rob1];
      tMat = xf1.R;
      var v1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var v1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tVec = vozlišča2[indeks];
      tMat = xf2.R;
      var v2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var v2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      v2X -= v1X;
      v2Y -= v1Y;
      ločitev var. = v2X * normal1WorldX + v2Y * normal1WorldY;
      povratna ločitev;
    };
    b2Collision.FindMaxSeparation = funkcija (edgeIndex, poly1, xf1, poly2, xf2) {
      var count1 = parseInt(poly1.m_vertexCount);
      var normals1 = poly1.m_normals;
      var tVec;
      var tMat;
      tMat = xf2.R;
      tVec = poly2.m_centroid;
      var dX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var dY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tMat = xf1.R;
      tVec = poly1.m_centroid;
      dX -= xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      dY -= xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      var dLocal1X = dX * xf1.R.col1.x + dY * xf1.R.col1.y;
      var dLocal1Y = dX * xf1.R.col2.x + dY * xf1.R.col2.y;
      var rob = 0;
      var maxDot = -Number.MAX_VALUE;
      for (var i = 0; i < count1; ++i) {
        tVec = normals1[i];
        var dot = tVec.x * dLocal1X + tVec.y * dLocal1Y;
        if (pika > maxDot) {
          maxDot = pika;
          rob = i;
        }
      }
      var s = b2Collision.EdgeSeparation(poli1, xf1, rob, poli2, xf2);
      var prevEdge = parseInt(edge ​​- 1 >= 0 ? edge - 1 : count1 - 1);
      var sPrev = b2Collision.EdgeSeparation(poli1, xf1, prevEdge, poli2, xf2);
      var nextEdge = parseInt(edge ​​+ 1 < count1 ? edge + 1 : 0);
      var sNext = b2Collision.EdgeSeparation(poli1, xf1, naslednjiRob, poli2, xf2);
      var bestEdge = 0;
      var bestSeparation = 0;
      var prirast = 0;
      if (sPrejšnji > s && sPrejšnji > sNaslednji) {
        prirastek = -1;
        bestEdge = prevEdge;
        bestSeparation = sPrev;
      } else if (sNaslednji > s) {
        prirastek = 1;
        bestEdge = nextEdge;
        najboljša ločitev = sNaslednji;
      } drugače {
        edgeIndex[0] = rob;
        vrni s;
      }
      medtem ko (true) {
        if (inkrement == -1) edge = bestEdge - 1 >= 0? bestEdge - 1 : štetje1 - 1;
        else edge = bestEdge + 1 < count1? bestEdge + 1 : 0;
        s = b2Collision.EdgeSeparation(poli1, xf1, rob, poli2, xf2);
        if (s > bestSeparation) {
          bestEdge = rob;
          najboljša ločitev = s;
        } drugače {
          odmor;
        }
      }
      edgeIndex[0] = bestEdge;
      vrni najboljšo ločitev;
    };
    b2Collision.FindIncidentEdge = funkcija (c, poli1, xf1, rob1, poli2, xf2) {
      če (rob1 === nedefiniran) rob1 = 0;
      var count1 = parseInt(poly1.m_vertexCount);
      var normals1 = poly1.m_normals;
      var count2 = parseInt(poly2.m_vertexCount);
      var vertices2 = poly2.m_vertices;
      var normals2 = poly2.m_normals;
      var tMat;
      var tVec;
      tMat = xf1.R;
      tVec = normals1[edge1];
      var normal1X = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      var normal1Y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      tMat = xf2.R;
      var tX = tMat.col1.x * normal1X + tMat.col1.y * normal1Y;
      normal1Y = tMat.col2.x * normal1X + tMat.col2.y * normal1Y;
      normalno1X = tX;
      spremenljiv indeks = 0;
      var minDot = Number.MAX_VALUE;
      for (var i = 0; i < count2; ++i) {
        tVec = normals2[i];
        var dot = normal1X * tVec.x + normal1Y * tVec.y;
        if (dot < minDot) {
          minDot = pika;
          indeks = i;
        }
      }
      var tClip;
      var i1 = parseInt(index);
      var i2 = parseInt(i1 + 1 < count2 ? i1 + 1 : 0);
      tClip = c[0];
      tVec = vozlišča2[i1];
      tMat = xf2.R;
      tClip.vx = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      tClip.vy = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tClip.id.features.referenceEdge = edge1;
      tClip.id.features.incidentEdge = i1;
      tClip.id.features.incidentVertex = 0;
      tClip = c[1];
      tVec = vozlišča2[i2];
      tMat = xf2.R;
      tClip.vx = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      tClip.vy = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tClip.id.features.referenceEdge = edge1;
      tClip.id.features.incidentEdge = i2;
      tClip.id.features.incidentVertex = 1;
    };
    b2Collision.MakeClipPointVector = funkcija () {
      var r = nov vektor(2);
      r[0] = novo ClipVertex();
      r[1] = novo ClipVertex();
      vrnitev r;
    };
    b2Collision.CollidePolygons = funkcija (množica, polyA, xfA, polyB, xfB) {
      var cv;
      manifold.m_pointCount = 0;
      var totalRadius = polyA.m_radius + polyB.m_radius;
      var edgeA = 0;
      b2Collision.s_edgeAO[0] = edgeA;
      var separationA = b2Collision.FindMaxSeparation(
        b2Collision.s_edgeAO,
        poliA,
        xfA,
        poliB,
        xfB
      );
      edgeA = b2Collision.s_edgeAO[0];
      if (separationA > totalRadius) return;
      var edgeB = 0;
      b2Collision.s_edgeBO[0] = robB;
      var separationB = b2Collision.FindMaxSeparation(
        b2Collision.s_edgeBO,
        poliB,
        xfB,
        poliA,
        xfA
      );
      edgeB = b2Collision.s_edgeBO[0];
      if (separationB > totalRadius) return;
      var poli1;
      var poli2;
      var xf1;
      var xf2;
      var rob1 = 0;
      var flip = 0;
      var k_relativeTol = 0,98;
      var k_absoluteTol = 0,001;
      var tMat;
      if (separationB > k_relativeTol * separationA + k_absoluteTol) {
        poli1 = poliB;
        poli2 = poliA;
        xf1 = xfB;
        xf2 = xfA;
        rob1 = robB;
        manifold.m_type = b2Manifold.e_faceB;
        preklop = 1;
      } drugače {
        poli1 = poliA;
        poli2 = poliB;
        xf1 = xfA;
        xf2 = xfB;
        rob1 = robA;
        manifold.m_type = b2Manifold.e_faceA;
        preklop = 0;
      }
      var incidentEdge = b2Collision.s_incidentEdge;
      b2Collision.FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);
      var count1 = parseInt(poly1.m_vertexCount);
      var vertices1 = poly1.m_vertices;
      var local_v11 = vozlišča1[rob1];
      var local_v12;
      if (edge1 + 1 < count1) {
        local_v12 = vozlišča1[parseInt(edge1 + 1)];
      } drugače {
        lokalna_v12 = vozlišča1[0];
      }
      var localTangent = b2Collision.s_localTangent;
      localTangent.Set(local_v12.x - local_v11.x, local_v12.y - local_v11.y);
      localTangent.Normalize();
      var localNormal = b2Collision.s_localNormal;
      localNormal.x = localTangent.y;
      localNormal.y = -localTangent.x;
      var planePoint = b2Collision.s_planePoint;
      planePoint.Set(
        0,5 * (local_v11.x + local_v12.x),
        0,5 * (local_v11.y + local_v12.y)
      );
      var tangent = b2Collision.s_tangent;
      tMat = xf1.R;
      tangent.x = tMat.col1.x * localTangent.x + tMat.col2.x * localTangent.y;
      tangent.y = tMat.col1.y * localTangent.x + tMat.col2.y * localTangent.y;
      var tangent2 = b2Collision.s_tangent2;
      tangenta2.x = -tangenta.x;
      tangenta2.y = -tangenta.y;
      var normal = b2Collision.s_normal;
      normal.x = tangenta.y;
      normal.y = -tangenta.x;
      var v11 = b2Collision.s_v11;
      var v12 = b2Collision.s_v12;
      v11.x =
        xf1.position.x + (tMat.col1.x * local_v11.x + tMat.col2.x * local_v11.y);
      v11.y =
        xf1.position.y + (tMat.col1.y * local_v11.x + tMat.col2.y * local_v11.y);
      v12.x =
        xf1.position.x + (tMat.col1.x * local_v12.x + tMat.col2.x * local_v12.y);
      v12.y =
        xf1.position.y + (tMat.col1.y * local_v12.x + tMat.col2.y * local_v12.y);
      var frontOffset = normal.x * v11.x + normal.y * v11.y;
      var sideOffset1 = -tangent.x * v11.x - tangent.y * v11.y + totalRadius;
      var sideOffset2 = tangent.x * v12.x + tangent.y * v12.y + totalRadius;
      var clipPoints1 = b2Collision.s_clipPoints1;
      var clipPoints2 = b2Collision.s_clipPoints2;
      var np = 0;
      np = b2Collision.ClipSegmentToLine(
        clipPoints1,
        incidentEdge,
        tangenta2,
        sideOffset1
      );
      če (np < 2) vrnitev;
      np = b2Collision.ClipSegmentToLine(
        clipPoints2,
        clipPoints1,
        tangenta,
        sideOffset2
      );
      če (np < 2) vrnitev;
      manifold.m_localPlaneNormal.SetV(localNormal);
      manifold.m_localPoint.SetV(planePoint);
      var pointCount = 0;
      for (var i = 0; i < b2Settings.b2_maxManifoldPoints; ++i) {
        cv = clipPoints2[i];
        var separation = normal.x * cv.vx + normal.y * cv.vy - frontOffset;
        if (ločitev <= totalRadius) {
          var cp = manifold.m_points[pointCount];
          tMat = xf2.R;
          var tX = cv.vx - xf2.position.x;
          var tY = cv.vy - xf2.position.y;
          cp.m_localPoint.x = tX * tMat.col1.x + tY * tMat.col1.y;
          cp.m_localPoint.y = tX * tMat.col2.x + tY * tMat.col2.y;
          cp.m_id.Set(cv.id);
          cp.m_id.features.flip = preklop;
          ++pointCount;
        }
      }
      manifold.m_pointCount = število točk;
    };
    b2Collision.CollideCircles = funkcija (množica, krog1, xf1, krog2, xf2) {
      manifold.m_pointCount = 0;
      var tMat;
      var tVec;
      tMat = xf1.R;
      tVec = krog1.m_p;
      var p1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var p1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      tMat = xf2.R;
      tVec = krog2.m_p;
      var p2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var p2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      var dX = p2X - p1X;
      var dY = p2Y - p1Y;
      var distSqr = dX * dX + dY * dY;
      var polmer = krog1.m_polmer + krog2.m_polmer;
      if (distSqr > polmer * polmer) {
        vrnitev;
      }
      manifold.m_type = b2Manifold.e_circles;
      manifold.m_localPoint.SetV(circle1.m_p);
      manifold.m_localPlaneNormal.SetZero();
      manifold.m_pointCount = 1;
      manifold.m_points[0].m_localPoint.SetV(circle2.m_p);
      manifold.m_points[0].m_id.key = 0;
    };
    b2Collision.CollidePolygonAndCircle = funkcija (
      razdelilnik,
      poligon,
      xf1,
      krog,
      xf2
    ) {
      manifold.m_pointCount = 0;
      var tPoint;
      var dX = 0;
      var dY = 0;
      var. položaj X = 0;
      var. položaj Y = 0;
      var tVec;
      var tMat;
      tMat = xf2.R;
      tVec = krog.m_p;
      var cX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var cY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      dX = cX - xf1.položaj.x;
      dY = cY - xf1.položaj.y;
      tMat = xf1.R;
      var cLocalX = dX * tMat.col1.x + dY * tMat.col1.y;
      var cLocalY = dX * tMat.col2.x + dY * tMat.col2.y;
      var dist = 0;
      var normalIndex = 0;
      var separation = -Number.MAX_VALUE;
      var radius = polygon.m_radius + circle.m_radius;
      var vertexCount = parseInt(polygon.m_vertexCount);
      var vertices = polygon.m_vertices;
      var normals = polygon.m_normals;
      for (var i = 0; i < vertexCount; ++i) {
        tVec = vozlišča[i];
        dX = cLocalX - tVec.x;
        dY = cLocalY - tVec.y;
        tVec = normali[i];
        var s = tVec.x * dX + tVec.y * dY;
        if (s > polmer) {
          vrnitev;
        }
        če (s > ločitev) {
          ločitev = s;
          normalIndex = i;
        }
      }
      var vertIndex1 = parseInt(normalIndex);
      var vertIndex2 = parseInt(
        vertIndex1 + 1 < vertexCount? vertIndex1 + 1 : 0
      );
      var v1 = vozlišča [vertIndex1];
      var v2 = vozlišča [vertIndex2];
      if (ločitev < število.MIN_VALUE) {
        manifold.m_pointCount = 1;
        manifold.m_type = b2Manifold.e_faceA;
        manifold.m_localPlaneNormal.SetV(normals[normalIndex]);
        manifold.m_localPoint.x = 0,5 * (v1.x + v2.x);
        manifold.m_localPoint.y = 0,5 * (v1.y + v2.y);
        manifold.m_points[0].m_localPoint.SetV(circle.m_p);
        manifold.m_points[0].m_id.key = 0;
        vrnitev;
      }
      var u1 =
        (cLocalX - v1.x) * (v2.x - v1.x) + (cLocalY - v1.y) * (v2.y - v1.y);
      var u2 =
        (cLocalX - v2.x) * (v1.x - v2.x) + (cLocalY - v2.y) * (v1.y - v2.y);
      če (u1 <= 0,0) {
        če (
          (cLocalX - v1.x) * (cLocalX - v1.x) +
            (cLocalY - v1.y) * (cLocalY - v1.y) >
          polmer * polmer
        )
          vrnitev;
        manifold.m_pointCount = 1;
        manifold.m_type = b2Manifold.e_faceA;
        manifold.m_localPlaneNormal.x = cLocalX - v1.x;
        manifold.m_localPlaneNormal.y = cLocalY - v1.y;
        manifold.m_localPlaneNormal.Normalize();
        manifold.m_localPoint.SetV(v1);
        manifold.m_points[0].m_localPoint.SetV(circle.m_p);
        manifold.m_points[0].m_id.key = 0;
      } sicer če (u2 <= 0) {
        če (
          (cLocalX - v2.x) * (cLocalX - v2.x) +
            (cLocalY - v2.y) * (cLocalY - v2.y) >
          polmer * polmer
        )
          vrnitev;
        manifold.m_pointCount = 1;
        manifold.m_type = b2Manifold.e_faceA;
        manifold.m_localPlaneNormal.x = cLocalX - v2.x;
        manifold.m_localPlaneNormal.y = cLocalY - v2.y;
        manifold.m_localPlaneNormal.Normalize();
        manifold.m_localPoint.SetV(v2);
        manifold.m_points[0].m_localPoint.SetV(circle.m_p);
        manifold.m_points[0].m_id.key = 0;
      } drugače {
        var faceCenterX = 0,5 * (v1.x + v2.x);
        var faceCenterY = 0,5 * (v1.y + v2.y);
        ločitev =
          (cLocalX - faceCenterX) * normals[vertIndex1].x +
          (cLocalY - faceCenterY) * normals[vertIndex1].y;
        if (separacija > polmer) return;
        manifold.m_pointCount = 1;
        manifold.m_type = b2Manifold.e_faceA;
        manifold.m_localPlaneNormal.x = normals[vertIndex1].x;
        manifold.m_localPlaneNormal.y = normals[vertIndex1].y;
        manifold.m_localPlaneNormal.Normalize();
        manifold.m_localPoint.Set(faceCenterX, faceCenterY);
        manifold.m_points[0].m_localPoint.SetV(circle.m_p);
        manifold.m_points[0].m_id.key = 0;
      }
    };
    b2Collision.TestOverlap = funkcija (a, b) {
      var t1 = b.lowerBound;
      var t2 = a.upperBound;
      var d1X = t1.x - t2.x;
      var d1Y = t1.y - t2.y;
      t1 = a.lowerBound;
      t2 = b.zgornja meja;
      var d2X = t1.x - t2.x;
      var d2Y = t1.y - t2.y;
      if (d1X > 0,0 || d1Y > 0,0) vrni false;
      if (d2X > 0,0 || d2Y > 0,0) vrni false;
      vrni resnico;
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Collision.b2Collision.s_incidentEdge =
        b2Collision.MakeClipPointVector();
      Box2D.Collision.b2Collision.s_clipPoints1 =
        b2Collision.MakeClipPointVector();
      Box2D.Collision.b2Collision.s_clipPoints2 =
        b2Collision.MakeClipPointVector();
      Box2D.Collision.b2Collision.s_edgeAO = new Vector_a2j_Number(1);
      Box2D.Collision.b2Collision.s_edgeBO = ​​new Vector_a2j_Number(1);
      Box2D.Collision.b2Collision.s_localTangent = novo b2Vec2();
      Box2D.Collision.b2Collision.s_localNormal = novo b2Vec2();
      Box2D.Collision.b2Collision.s_planePoint = novo b2Vec2();
      Box2D.Collision.b2Collision.s_normal = novo b2Vec2();
      Box2D.Collision.b2Collision.s_tangent = novo b2Vec2();
      Box2D.Collision.b2Collision.s_tangent2 = novo b2Vec2();
      Box2D.Collision.b2Collision.s_v11 = novo b2Vec2();
      Box2D.Collision.b2Collision.s_v12 = novo b2Vec2();
      Box2D.Collision.b2Collision.b2CollidePolyTempVec = novo b2Vec2();
      Box2D.Collision.b2Collision.b2_nullFeature = 0x000000ff;
    });
    b2ContactID.b2ContactID = funkcija () {
      this.features = nove funkcije();
    };
    b2ContactID.prototype.b2ContactID = funkcija () {
      this.features._m_id = to;
    };
    b2ContactID.prototype.Set = funkcija (id) {
      this.key = id._key;
    };
    b2ContactID.prototype.Copy = funkcija () {
      var id = new b2ContactID();
      id.key = this.key;
      povratni ID;
    };
    Object.defineProperty(b2ContactID.prototype, "ključ", {
      enumerable: false,
      nastavljivo: res,
      get: funkcija () {
        vrni to._ključ;
      },
    });
    Object.defineProperty(b2ContactID.prototype, "ključ", {
      enumerable: false,
      nastavljivo: res,
      set: funkcija (vrednost) {
        če (vrednost === nedefinirano) vrednost = 0;
        this._key = vrednost;
        this.features._referenceEdge = this._key & 0x000000ff;
        this.features._incidentEdge =
          ((ta._ključ & 0x0000ff00) >> 8) & 0x000000ff;
        this.features._incidentVertex =
          ((ta._ključ & 0x00ff0000) >> 16) & 0x000000ff;
        this.features._flip = ((this._key & 0xff000000) >> 24) & 0x000000ff;
      },
    });
    b2ContactPoint.b2ContactPoint = funkcija () {
      this.position = new b2Vec2();
      this.velocity = novo b2Vec2();
      this.normal = novo b2Vec2();
      this.id = new b2ContactID();
    };
    b2Distance.b2Distance = funkcija () {};
    b2Distance.Distance = funkcija (izhod, predpomnilnik, vhod) {
      ++b2Distance.b2_gjkCalls;
      var proxyA = input.proxyA;
      var proxyB = input.proxyB;
      var transformA = input.transformA;
      var transformB = input.transformB;
      var simplex = b2Distance.s_simplex;
      simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);
      var vertices = simplex.m_vertices;
      var k_maxIters = 20;
      var saveA = b2Distance.s_saveA;
      var saveB = b2Distance.s_saveB;
      var saveCount = 0;
      var closestPoint = simplex.GetClosestPoint();
      var distanceSqr1 = closestPoint.LengthSquared();
      var distanceSqr2 = distanceSqr1;
      var i = 0;
      var p;
      var iter = 0;
      medtem ko (iter < k_maxIters) {
        saveCount = simplex.m_count;
        for (i = 0; i < saveCount; i++) {
          shraniA[i] = vozlišča[i].indexA;
          shraniB[i] = vozlišča[i].indexB;
        }
        stikalo (simplex.m_count) {
          primer 1:
            odmor;
          primer 2:
            simplex.Solve2();
            odmor;
          primer 3:
            simplex.Solve3();
            odmor;
          privzeto:
            b2Settings.b2Assert(false);
        }
        če (simplex.m_count == 3) {
          odmor;
        }
        p = simplex.GetClosestPoint();
        distanceSqr2 = p.LengthSquared();
        if (distanceSqr2 > distanceSqr1) {
        }
        razdaljaSqr1 = razdaljaSqr2;
        var d = simplex.GetSearchDirection();
        if (d.LengthSquared() < Number.MIN_VALUE * Number.MIN_VALUE) {
          odmor;
        }
        var vertex = vertices[simplex.m_count];
        vertex.indexA = proxyA.GetSupport(
          b2Math.MulTMV(transformA.R, d.GetNegative())
        );
        vertex.wA = b2Math.MulX(transformA, proxyA.GetVertex(vertex.indexA));
        vertex.indexB = proxyB.GetSupport(b2Math.MulTMV(transformB.R, d));
        vertex.wB = b2Math.MulX(transformB, proxyB.GetVertex(vertex.indexB));
        vertex.w = b2Math.SubtractVV(vertex.wB, vertex.wA);
        ++iter;
        ++b2Distance.b2_gjkIters;
        var duplicate = false;
        for (i = 0; i < saveCount; i++) {
          if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i]) {
            dvojnik = res;
            odmor;
          }
        }
        če (dvojnik) {
          odmor;
        }
        ++simplex.m_count;
      }
      b2Distance.b2_gjkMaxIters = b2Math.Max(b2Distance.b2_gjkMaxIters, iter);
      simplex.GetWitnessPoints(output.pointA, output.pointB);
      output.distance = b2Math.SubtractVV(output.pointA, output.pointB).Length();
      output.iterations = iter;
      simplex.WriteCache(predpomnilnik);
      if (input.useRadii) {
        var rA = proxyA.m_radius;
        var rB = proxyB.m_radius;
        if (output.distance > rA + rB && output.distance > Number.MIN_VALUE) {
          izhodna razdalja -= rA + rB;
          var normal = b2Math.SubtractVV(output.pointB, output.pointA);
          normalno.Normaliziraj();
          output.pointA.x += rA * normal.x;
          output.pointA.y += rA * normal.y;
          output.pointB.x -= rB * normal.x;
          output.pointB.y -= rB * normal.y;
        } drugače {
          p = novo b2Vec2();
          px = 0,5 * (output.pointA.x + output.pointB.x);
          py = 0,5 * (output.pointA.y + output.pointB.y);
          output.pointA.x = output.pointB.x = px;
          output.pointA.y = output.pointB.y = py;
          izhod.razdalja = 0,0;
        }
      }
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Collision.b2Distance.s_simplex = novo b2Simplex();
      Box2D.Collision.b2Distance.s_saveA = new Vector_a2j_Number(3);
      Box2D.Collision.b2Distance.s_saveB = new Vector_a2j_Number(3);
    });
    b2DistanceInput.b2DistanceInput = funkcija () {};
    b2DistanceOutput.b2DistanceOutput = funkcija () {
      this.pointA = novo b2Vec2();
      this.pointB = novo b2Vec2();
    };
    b2DistanceProxy.b2DistanceProxy = funkcija () {};
    b2DistanceProxy.prototype.Set = funkcija (oblika) {
      stikalo (shape.GetType()) {
        case b2Shape.e_circleShape:
          {
            var circle = shape instanceof b2CircleShape? oblika : nič;
            this.m_vertices = nov vektor (1, res);
            this.m_vertices[0] = circle.m_p;
            this.m_count = 1;
            this.m_radius = krog.m_radius;
          }
          odmor;
        primer b2Shape.e_polygonShape:
          {
            var polygon = shape instanceof b2PolygonShape? oblika : nič;
            this.m_vertices = polygon.m_vertices;
            this.m_count = polygon.m_vertexCount;
            this.m_radius = poligon.m_radius;
          }
          odmor;
        privzeto:
          b2Settings.b2Assert(false);
      }
    };
    b2DistanceProxy.prototype.GetSupport = funkcija (d) {
      var bestIndex = 0;
      var bestValue = this.m_vertices[0].x * dx + this.m_vertices[0].y * dy;
      for (var i = 1; i < this.m_count; ++i) {
        var value = this.m_vertices[i].x * dx + this.m_vertices[i].y * dy;
        if (value > bestValue) {
          bestIndex = i;
          bestValue = vrednost;
        }
      }
      vrni bestIndex;
    };
    b2DistanceProxy.prototype.GetSupportVertex = funkcija (d) {
      var bestIndex = 0;
      var bestValue = this.m_vertices[0].x * dx + this.m_vertices[0].y * dy;
      for (var i = 1; i < this.m_count; ++i) {
        var value = this.m_vertices[i].x * dx + this.m_vertices[i].y * dy;
        if (value > bestValue) {
          bestIndex = i;
          bestValue = vrednost;
        }
      }
      vrni this.m_vertices[bestIndex];
    };
    b2DistanceProxy.prototype.GetVertexCount = funkcija () {
      vrni this.m_count;
    };
    b2DistanceProxy.prototype.GetVertex = funkcija (indeks) {
      če (indeks === nedefiniran) indeks = 0;
      b2Settings.b2Assert(0 <= index && index < this.m_count);
      vrni this.m_vertices[index];
    };
    b2DynamicTree.b2DynamicTree = funkcija () {};
    b2DynamicTree.prototype.b2DynamicTree = funkcija () {
      this.m_root = null;
      this.m_freeList = null;
      this.m_path = 0;
      this.m_insertionCount = 0;
    };
    b2DynamicTree.prototype.CreateProxy = funkcija (aabb, uporabniški podatki) {
      var node = this.AllocateNode();
      var extendX = b2Settings.b2_aabbExtension;
      var extendY = b2Settings.b2_aabbExtension;
      node.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
      vozlišče.aabb.lowerBound.y = aabb.lowerBound.y - razširi Y;
      vozlišče.aabb.upperBound.x = aabb.upperBound.x + extendX;
      vozlišče.aabb.upperBound.y = aabb.upperBound.y + extendY;
      node.userData = uporabniški podatki;
      this.InsertLeaf(vozlišče);
      povratno vozlišče;
    };
    b2DynamicTree.prototype.DestroyProxy = funkcija (proxy) {
      this.RemoveLeaf(proxy);
      this.FreeNode(proxy);
    };
    b2DynamicTree.prototype.MoveProxy = funkcija (proxy, aabb, premik) {
      b2Settings.b2Assert(proxy.IsLeaf());
      if (proxy.aabb.Contains(aabb)) {
        vrni false;
      }
      this.RemoveLeaf(proxy);
      var extendX =
        b2Settings.b2_aabbExtension +
        b2Settings.b2_aabbMultiplier *
          (premik.x > 0 ? premik.x : -premik.x);
      var extendY =
        b2Settings.b2_aabbExtension +
        b2Settings.b2_aabbMultiplier *
          (premik.y > 0 ? premik.y : -premik.y);
      proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
      proxy.aabb.lowerBound.y = aabb.lowerBound.y - razširi Y;
      proxy.aabb.upperBound.x = aabb.upperBound.x + extendX;
      proxy.aabb.upperBound.y = aabb.upperBound.y + extendY;
      this.InsertLeaf(proxy);
      vrni resnico;
    };
    b2DynamicTree.prototype.Rebalance = funkcija (iteracije) {
      če (iteracije === nedefinirano) iteracije = 0;
      if (this.m_root == null) vrnitev;
      for (var i = 0; i < iterations; i++) {
        var node = this.m_root;
        var bit = 0;
        medtem ko (node.IsLeaf() == false) {
          vozlišče = (ta.m_pot >> bit) & 1? vozlišče.child2: vozlišče.child1;
          bit = (bit + 1) & 31;
        }
        ++this.m_path;
        this.RemoveLeaf(vozlišče);
        this.InsertLeaf(vozlišče);
      }
    };
    b2DynamicTree.prototype.GetFatAABB = funkcija (proxy) {
      vrni proxy.aabb;
    };
    b2DynamicTree.prototype.GetUserData = funkcija (proxy) {
      vrni proxy.userData;
    };
    b2DynamicTree.prototype.Query = funkcija (povratni klic, aabb) {
      if (this.m_root == null) vrnitev;
      var stack = new Vector();
      spremenljivo število = 0;
      stack[count++] = this.m_root;
      medtem ko (štetje > 0) {
        var node = stack[--count];
        if (node.aabb.TestOverlap(aabb)) {
          if (node.IsLeaf()) {
            var continue = callback(vozlišče);
            če (!proceed) vrnitev;
          } drugače {
            stack[count++] = node.child1;
            stack[count++] = node.child2;
          }
        }
      }
    };
    b2DynamicTree.prototype.RayCast = funkcija (povratni klic, vnos) {
      if (this.m_root == null) vrnitev;
      var p1 = input.p1;
      var p2 = input.p2;
      var r = b2Math.SubtractVV(p1, p2);
      r.Normaliziraj();
      var v = b2Math.CrossFV(1.0, r);
      var abs_v = b2Math.AbsV(v);
      var maxFraction = input.maxFraction;
      var segmentAABB = novo b2AABB();
      var tX = 0;
      var tY = 0;
      {
        tX = p1.x + maxFraction * (p2.x - p1.x);
        tY = p1.y + maxFraction * (p2.y - p1.y);
        segmentAABB.lowerBound.x = Math.min(p1.x, tX);
        segmentAABB.lowerBound.y = Math.min(p1.y, tY);
        segmentAABB.upperBound.x = Math.max(p1.x, tX);
        segmentAABB.upperBound.y = Math.max(p1.y, tY);
      }
      var stack = new Vector();
      spremenljivo število = 0;
      stack[count++] = this.m_root;
      medtem ko (štetje > 0) {
        var node = stack[--count];
        if (node.aabb.TestOverlap(segmentAABB) == false) {
          nadaljevati;
        }
        var c = vozlišče.aabb.GetCenter();
        var h = node.aabb.GetExtents();
        ločitev var. =
          Math.abs(vx * (p1.x - cx) + vy * (p1.y - cy)) -
          abs_v.x * hx -
          abs_v.y * hy;
        če (ločitev > 0,0) nadaljevanje;
        if (node.IsLeaf()) {
          var subInput = new b2RayCastInput();
          subInput.p1 = input.p1;
          subInput.p2 = input.p2;
          subInput.maxFraction = input.maxFraction;
          maxFraction = povratni klic(podvnos, vozlišče);
          if (maxFraction == 0.0) return;
          if (maxFraction > 0,0) {
            tX = p1.x + maxFraction * (p2.x - p1.x);
            tY = p1.y + maxFraction * (p2.y - p1.y);
            segmentAABB.lowerBound.x = Math.min(p1.x, tX);
            segmentAABB.lowerBound.y = Math.min(p1.y, tY);
            segmentAABB.upperBound.x = Math.max(p1.x, tX);
            segmentAABB.upperBound.y = Math.max(p1.y, tY);
          }
        } drugače {
          stack[count++] = node.child1;
          stack[count++] = node.child2;
        }
      }
    };
    b2DynamicTree.prototype.AllocateNode = funkcija () {
      if (this.m_freeList) {
        var node = this.m_freeList;
        this.m_freeList = node.parent;
        node.parent = null;
        vozlišče.child1 = nič;
        node.child2 = nič;
        povratno vozlišče;
      }
      vrni novo b2DynamicTreeNode();
    };
    b2DynamicTree.prototype.FreeNode = funkcija (vozlišče) {
      node.parent = this.m_freeList;
      this.m_freeList = vozlišče;
    };
    b2DynamicTree.prototype.InsertLeaf = funkcija (list) {
      ++this.m_insertionCount;
      if (this.m_root == null) {
        this.m_root = list;
        this.m_root.parent = null;
        vrnitev;
      }
      var center = leaf.aabb.GetCenter();
      var sibling = this.m_root;
      if (sibling.IsLeaf() == false) {
        narediti {
          var otrok1 = sorojenec.otrok1;
          var otrok2 = sorojenec.otrok2;
          var norma1 =
            Math.abs(
              (child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x
            ) +
            Math.abs(
              (child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y
            );
          var norm2 =
            Math.abs(
              (child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x
            ) +
            Math.abs(
              (child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y
            );
          če (norma1 < norma2) {
            sorojenec = otrok1;
          } drugače {
            sorojenec = otrok2;
          }
        } medtem ko (sibling.IsLeaf() == false);
      }
      var node1 = sibling.parent;
      var node2 = this.AllocateNode();
      vozlišče2.nadrejeni = vozlišče1;
      node2.userData = null;
      vozlišče2.aabb.Združi(list.aabb, sorodnik.aabb);
      če (vozlišče1) {
        if (sibling.parent.child1 == sorojenec) {
          vozlišče1.otrok1 = vozlišče2;
        } drugače {
          vozlišče1.otrok2 = vozlišče2;
        }
        vozlišče2.child1 = sorojenec;
        vozlišče2.otrok2 = list;
        sibling.parent = vozlišče2;
        leaf.parent = vozlišče2;
        narediti {
          if (node1.aabb.Contain(node2.aabb)) break;
          vozlišče1.aabb.Združi (vozlišče1.child1.aabb, node1.child2.aabb);
          vozlišče2 = vozlišče1;
          vozlišče1 = vozlišče1.nadrejeni;
        } medtem ko (vozlišče1);
      } drugače {
        vozlišče2.child1 = sorojenec;
        vozlišče2.otrok2 = list;
        sibling.parent = vozlišče2;
        leaf.parent = vozlišče2;
        this.m_root = vozlišče2;
      }
    };
    b2DynamicTree.prototype.RemoveLeaf = funkcija (list) {
      če (list == this.m_root) {
        this.m_root = null;
        vrnitev;
      }
      var node2 = leaf.parent;
      var node1 = node2.parent;
      var sorodnik;
      if (node2.child1 == list) {
        brat = vozlišče2.otrok2;
      } drugače {
        brat = vozlišče2.otrok1;
      }
      če (vozlišče1) {
        if (node1.child1 == node2) {
          vozlišče1.child1 = sorojenec;
        } drugače {
          vozlišče1.child2 = sorojenec;
        }
        sibling.parent = vozlišče1;
        this.FreeNode(node2);
        medtem ko (vozlišče1) {
          var oldAABB = vozlišče1.aabb;
          node1.aabb = b2AABB.Combine(node1.child1.aabb, node1.child2.aabb);
          if (oldAABB.Contains(node1.aabb)) break;
          vozlišče1 = vozlišče1.nadrejeni;
        }
      } drugače {
        this.m_root = sorojenec;
        sibling.parent = null;
        this.FreeNode(node2);
      }
    };
    b2DynamicTreeBroadPhase.b2DynamicTreeBroadPhase = funkcija () {
      this.m_tree = novo b2DynamicTree();
      this.m_moveBuffer = nov vektor();
      this.m_pairBuffer = nov vektor();
      this.m_pairCount = 0;
    };
    b2DynamicTreeBroadPhase.prototype.CreateProxy = funkcija (aabb, uporabniški podatki) {
      var proxy = this.m_tree.CreateProxy(aabb, userData);
      ++this.m_proxyCount;
      this.BufferMove(proxy);
      povratni proxy;
    };
    b2DynamicTreeBroadPhase.prototype.DestroyProxy = funkcija (proxy) {
      this.UnBufferMove(proxy);
      --this.m_proxyCount;
      this.m_tree.DestroyProxy(proxy);
    };
    b2DynamicTreeBroadPhase.prototype.MoveProxy = funkcija (
      zastopnik,
      aabb,
      premik
    ) {
      var buffer = this.m_tree.MoveProxy(proxy, aabb, displacement);
      if (medpomnilnik) {
        this.BufferMove(proxy);
      }
    };
    b2DynamicTreeBroadPhase.prototype.TestOverlap = funkcija (proxyA, proxyB) {
      var aabbA = this.m_tree.GetFatAABB(proxyA);
      var aabbB = this.m_tree.GetFatAABB(proxyB);
      vrni aabbA.TestOverlap(aabbB);
    };
    b2DynamicTreeBroadPhase.prototype.GetUserData = funkcija (proxy) {
      vrni this.m_tree.GetUserData(proxy);
    };
    b2DynamicTreeBroadPhase.prototype.GetFatAABB = funkcija (proxy) {
      vrni this.m_tree.GetFatAABB(proxy);
    };
    b2DynamicTreeBroadPhase.prototype.GetProxyCount = funkcija () {
      vrni this.m_proxyCount;
    };
    b2DynamicTreeBroadPhase.prototype.UpdatePairs = funkcija (povratni klic) {
      var __this = to;
      __this.m_pairCount = 0;
      var i = 0,
        queryProxy;
      funkcija QueryCallback(proxy) {
        if (proxy == queryProxy) vrne true;
        if (__this.m_pairCount == __this.m_pairBuffer.length) {
          __this.m_pairBuffer[__this.m_pairCount] = nov b2DynamicTreePair();
        }
        var pair = __this.m_pairBuffer[__this.m_pairCount];
        pair.proxyA = proxy < queryProxy? proxy : queryProxy;
        pair.proxyB = proxy >= queryProxy? proxy : queryProxy;
        ++__this.m_pairCount;
        vrni resnico;
      }
      for (i = 0; i < __this.m_moveBuffer.length; ++i) {
        queryProxy = __this.m_moveBuffer[i];
        var fatAABB = __this.m_tree.GetFatAABB(queryProxy);
        __this.m_tree.Query(QueryCallback, fatAABB);
      }
      __this.m_moveBuffer.length = 0;
      for (var i = 0; i < __this.m_pairCount; ) {
        var primarniPair = __this.m_pairBuffer[i];
        var userDataA = __this.m_tree.GetUserData(primaryPair.proxyA);
        var userDataB = __this.m_tree.GetUserData(primaryPair.proxyB);
        povratni klic(uporabniškipodatkiA, uporabniškipodatkiB);
        ++i;
        medtem ko (i < __this.m_pairCount) {
          var pair = __this.m_pairBuffer[i];
          če (
            pair.proxyA != primarniPair.proxyA ||
            pair.proxyB != primarniPair.proxyB
          ) {
            odmor;
          }
          ++i;
        }
      }
    };
    b2DynamicTreeBroadPhase.prototype.Query = funkcija (povratni klic, aabb) {
      this.m_tree.Query(callback, aabb);
    };
    b2DynamicTreeBroadPhase.prototype.RayCast = funkcija (povratni klic, vnos) {
      this.m_tree.RayCast(povratni klic, vnos);
    };
    b2DynamicTreeBroadPhase.prototype.Validate = funkcija () {};
    b2DynamicTreeBroadPhase.prototype.Rebalance = funkcija (iteracije) {
      če (iteracije === nedefinirano) iteracije = 0;
      this.m_tree.Rebalance(iterations);
    };
    b2DynamicTreeBroadPhase.prototype.BufferMove = funkcija (proxy) {
      this.m_moveBuffer[this.m_moveBuffer.length] = proxy;
    };
    b2DynamicTreeBroadPhase.prototype.UnBufferMove = funkcija (proxy) {
      var i = parseInt(this.m_moveBuffer.indexOf(proxy));
      this.m_moveBuffer.splice(i, 1);
    };
    b2DynamicTreeBroadPhase.prototype.ComparePairs = funkcija (par1, par2) {
      vrni 0;
    };
    b2DynamicTreeBroadPhase.__implements = {};
    b2DynamicTreeBroadPhase.__implements[IBroadPhase] = true;
    b2DynamicTreeNode.b2DynamicTreeNode = funkcija () {
      this.aabb = novo b2AABB();
    };
    b2DynamicTreeNode.prototype.IsLeaf = funkcija () {
      vrni this.child1 == null;
    };
    b2DynamicTreePair.b2DynamicTreePair = funkcija () {};
    b2Manifold.b2Manifold = funkcija () {
      this.m_pointCount = 0;
    };
    b2Manifold.prototype.b2Manifold = funkcija () {
      this.m_points = nov vektor(b2Settings.b2_maxManifoldPoints);
      for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
        this.m_points[i] = new b2ManifoldPoint();
      }
      this.m_localPlaneNormal = novo b2Vec2();
      this.m_localPoint = novo b2Vec2();
    };
    b2Manifold.prototype.Reset = funkcija () {
      for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
        (this.m_points[i] primerek b2ManifoldPoint
          ? this.m_points[i]
          : nič
        ).Ponastaviti();
      }
      this.m_localPlaneNormal.SetZero();
      this.m_localPoint.SetZero();
      this.m_type = 0;
      this.m_pointCount = 0;
    };
    b2Manifold.prototype.Set = funkcija (m) {
      this.m_pointCount = m.m_pointCount;
      for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
        (this.m_points[i] primerek b2ManifoldPoint
          ? this.m_points[i]
          : nič
        ).Set(m.m_points[i]);
      }
      this.m_localPlaneNormal.SetV(m.m_localPlaneNormal);
      this.m_localPoint.SetV(m.m_localPoint);
      this.m_type = m.m_type;
    };
    b2Manifold.prototype.Copy = funkcija () {
      var copy = new b2Manifold();
      copy.Set(this);
      povratni izvod;
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Collision.b2Manifold.e_circles = 0x0001;
      Box2D.Collision.b2Manifold.e_faceA = 0x0002;
      Box2D.Collision.b2Manifold.e_faceB = 0x0004;
    });
    b2ManifoldPoint.b2ManifoldPoint = funkcija () {
      this.m_localPoint = novo b2Vec2();
      this.m_id = new b2ContactID();
    };
    b2ManifoldPoint.prototype.b2ManifoldPoint = funkcija () {
      this.Reset();
    };
    b2ManifoldPoint.prototype.Reset = funkcija () {
      this.m_localPoint.SetZero();
      this.m_normalImpulse = 0,0;
      this.m_tangentImpulse = 0,0;
      this.m_id.key = 0;
    };
    b2ManifoldPoint.prototype.Set = funkcija (m) {
      this.m_localPoint.SetV(m.m_localPoint);
      this.m_normalniimpulz = m.m_normalniimpulz;
      this.m_tangentni impulz = m.m_tangentni impulz;
      this.m_id.Set(m.m_id);
    };
    b2Point.b2Point = funkcija () {
      this.p = novo b2Vec2();
    };
    b2Point.prototype.Support = funkcija (xf, vX, vY) {
      if (vX === nedefinirano) vX = 0;
      if (vY === nedefinirano) vY = 0;
      vrni to.p;
    };
    b2Point.prototype.GetFirstVertex = funkcija (xf) {
      vrni to.p;
    };
    b2RayCastInput.b2RayCastInput = funkcija () {
      this.p1 = novo b2Vec2();
      this.p2 = novo b2Vec2();
    };
    b2RayCastInput.prototype.b2RayCastInput = funkcija (p1, p2, maxFraction) {
      če (p1 === nedefinirano) p1 = nič;
      če (p2 === nedefinirano) p2 = nič;
      if (maxFraction === nedefinirano) maxFraction = 1;
      if (p1) this.p1.SetV(p1);
      if (p2) this.p2.SetV(p2);
      this.maxFraction = maxFraction;
    };
    b2RayCastOutput.b2RayCastOutput = funkcija () {
      this.normal = novo b2Vec2();
    };
    b2Segment.b2Segment = funkcija () {
      this.p1 = novo b2Vec2();
      this.p2 = novo b2Vec2();
    };
    b2Segment.prototype.TestSegment = funkcija (
      lambda,
      normalno,
      segment,
      maxLambda
    ) {
      if (maxLambda === nedefinirano) maxLambda = 0;
      var s = segment.p1;
      var rX = segment.p2.x - sx;
      var rY = segment.p2.y - sy;
      var dX = this.p2.x - this.p1.x;
      var dY = this.p2.y - this.p1.y;
      var nX = dY;
      var nY = -dX;
      var k_slop = 100,0 * Število.MIN_VREDNOST;
      var denom = -(rX * nX + rY * nY);
      if (denom > k_slop) {
        var bX = sx - this.p1.x;
        var bY = sy - this.p1.y;
        var a = bX * nX + bY * nY;
        if (0,0 <= a && a <= maxLambda * denom) {
          var mu2 = -rX * bY + rY * bX;
          if (-k_slop * denom <= mu2 && mu2 <= denom * (1,0 + k_slop)) {
            a /= denom;
            var nLen = Math.sqrt(nX * nX + nY * nY);
            nX /= nLen;
            nY /= nLen;
            lambda[0] = a;
            normal.Set(nX, nY);
            vrni resnico;
          }
        }
      }
      vrni false;
    };
    b2Segment.prototype.Extend = funkcija (aabb) {
      this.ExtendForward(aabb);
      this.ExtendBackward(aabb);
    };
    b2Segment.prototype.ExtendForward = funkcija (aabb) {
      var dX = this.p2.x - this.p1.x;
      var dY = this.p2.y - this.p1.y;
      var lambda = Math.min(
        dX > 0
          ? (aabb.upperBound.x - this.p1.x) / dX
          : dX < 0
          ? (aabb.lowerBound.x - this.p1.x) / dX
          : Število.POSITIVE_INFINITY,
        dY > 0
          ? (aabb.upperBound.y - this.p1.y) / dY
          : dY < 0
          ? (aabb.lowerBound.y - this.p1.y) / dY
          : Število.POZITIVNA_NESKONČNOST
      );
      this.p2.x = this.p1.x + dX * lambda;
      this.p2.y = this.p1.y + dY * lambda;
    };
    b2Segment.prototype.ExtendBackward = funkcija (aabb) {
      var dX = -this.p2.x + this.p1.x;
      var dY = -this.p2.y + this.p1.y;
      var lambda = Math.min(
        dX > 0
          ? (aabb.upperBound.x - this.p2.x) / dX
          : dX < 0
          ? (aabb.lowerBound.x - this.p2.x) / dX
          : Število.POSITIVE_INFINITY,
        dY > 0
          ? (aabb.upperBound.y - this.p2.y) / dY
          : dY < 0
          ? (aabb.lowerBound.y - this.p2.y) / dY
          : Število.POZITIVNA_NESKONČNOST
      );
      this.p1.x = this.p2.x + dX * lambda;
      this.p1.y = this.p2.y + dY * lambda;
    };
    b2SeparationFunction.b2SeparationFunction = funkcija () {
      this.m_localPoint = novo b2Vec2();
      this.m_axis = new b2Vec2();
    };
    b2SeparationFunction.prototype.Initialize = funkcija (
      predpomnilnik,
      proxyA,
      transformA,
      proxyB,
      transformacijaB
    ) {
      this.m_proxyA = proxyA;
      this.m_proxyB = proxyB;
      var count = parseInt(cache.count);
      b2Settings.b2Assert(0 < count && count < 3);
      var localPointA;
      var localPointA1;
      var localPointA2;
      var localPointB;
      var localPointB1;
      var localPointB2;
      var pointAX = 0;
      var pointAY = 0;
      var pointBX = 0;
      var pointBY = 0;
      var normalX = 0;
      var normalnoY = 0;
      var tMat;
      var tVec;
      var s = 0;
      var sgn = 0;
      če (štetje == 1) {
        this.m_type = b2SeparationFunction.e_points;
        localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
        localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
        tVec = lokalna točkaA;
        tMat = transformA.R;
        točkaAX =
          transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        točkaAY =
          transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tVec = lokalna točkaB;
        tMat = transformB.R;
        točkaBX =
          transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        točkaBY =
          transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        this.m_axis.x = pointBX - pointAX;
        this.m_axis.y = pointBY - pointAY;
        this.m_axis.Normalize();
      } else if (cache.indexB[0] == cache.indexB[1]) {
        this.m_type = b2SeparationFunction.e_faceA;
        localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
        localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
        localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
        this.m_localPoint.x = 0,5 * (localPointA1.x + localPointA2.x);
        this.m_localPoint.y = 0,5 * (localPointA1.y + localPointA2.y);
        this.m_axis = b2Math.CrossVF(
          b2Math.SubtractVV(localPointA2, localPointA1),
          1.0
        );
        this.m_axis.Normalize();
        tVec = this.m_os;
        tMat = transformA.R;
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tVec = this.m_localPoint;
        tMat = transformA.R;
        točkaAX =
          transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        točkaAY =
          transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tVec = lokalna točkaB;
        tMat = transformB.R;
        točkaBX =
          transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        točkaBY =
          transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        s = (točkaBX - točkaAX) * normalX + (pointBY - točkaAY) * normalY;
        če (s < 0,0) {
          this.m_axis.NegativeSelf();
        }
      } else if (cache.indexA[0] == cache.indexA[0]) {
        this.m_type = b2SeparationFunction.e_faceB;
        localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
        localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
        localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
        this.m_localPoint.x = 0,5 * (localPointB1.x + localPointB2.x);
        this.m_localPoint.y = 0,5 * (localPointB1.y + localPointB2.y);
        this.m_axis = b2Math.CrossVF(
          b2Math.SubtractVV(localPointB2, localPointB1),
          1.0
        );
        this.m_axis.Normalize();
        tVec = this.m_os;
        tMat = transformB.R;
        normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tVec = this.m_localPoint;
        tMat = transformB.R;
        točkaBX =
          transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        točkaBY =
          transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        tVec = lokalna točkaA;
        tMat = transformA.R;
        točkaAX =
          transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        točkaAY =
          transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        s = (točkaAX - točkaBX) * normalX + (točkaAY - točkaBY) * normalY;
        če (s < 0,0) {
          this.m_axis.NegativeSelf();
        }
      } drugače {
        localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
        localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
        localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
        localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
        var pA = b2Math.MulX(transformA, localPointA);
        var dA = b2Math.MulMV(
          transformA.R,
          b2Math.SubtractVV(localPointA2, localPointA1)
        );
        var pB = b2Math.MulX(transformB, localPointB);
        var dB = b2Math.MulMV(
          transformirajB.R,
          b2Math.SubtractVV(localPointB2, localPointB1)
        );
        var a = dA.x * dA.x + dA.y * dA.y;
        var e = dB.x * dB.x + dB.y * dB.y;
        var r = b2Math.SubtractVV(dB, dA);
        var c = dA.x * rx + dA.y * ry;
        var f = dB.x * rx + dB.y * ry;
        var b = dA.x * dB.x + dA.y * dB.y;
        var denom = a * e - b * b;
        s = 0,0;
        če (denom != 0,0) {
          s = b2Math.Clamp((b * f - c * e) / denom, 0,0, 1,0);
        }
        var t = (b * s + f) / e;
        če (t < 0,0) {
          t = 0,0;
          s = b2Math.Clamp((b - c) / a, 0,0, 1,0);
        }
        localPointA = novo b2Vec2();
        localPointA.x = localPointA1.x + s * (localPointA2.x - localPointA1.x);
        localPointA.y = localPointA1.y + s * (localPointA2.y - localPointA1.y);
        localPointB = novo b2Vec2();
        localPointB.x = localPointB1.x + s * (localPointB2.x - localPointB1.x);
        localPointB.y = localPointB1.y + s * (localPointB2.y - localPointB1.y);
        če (s == 0,0 || s == 1,0) {
          this.m_type = b2SeparationFunction.e_faceB;
          this.m_axis = b2Math.CrossVF(
            b2Math.SubtractVV(localPointB2, localPointB1),
            1.0
          );
          this.m_axis.Normalize();
          this.m_localPoint = localPointB;
          tVec = this.m_os;
          tMat = transformB.R;
          normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
          normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
          tVec = this.m_localPoint;
          tMat = transformB.R;
          točkaBX =
            transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
          točkaBY =
            transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
          tVec = lokalna točkaA;
          tMat = transformA.R;
          točkaAX =
            transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
          točkaAY =
            transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
          sgn = (točkaAX - točkaBX) * normalX + (točkaAY - točkaBY) * normalY;
          če (s < 0,0) {
            this.m_axis.NegativeSelf();
          }
        } drugače {
          this.m_type = b2SeparationFunction.e_faceA;
          this.m_axis = b2Math.CrossVF(
            b2Math.SubtractVV(localPointA2, localPointA1),
            1.0
          );
          this.m_localPoint = localPointA;
          tVec = this.m_os;
          tMat = transformA.R;
          normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
          normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
          tVec = this.m_localPoint;
          tMat = transformA.R;
          točkaAX =
            transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
          točkaAY =
            transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
          tVec = lokalna točkaB;
          tMat = transformB.R;
          točkaBX =
            transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
          točkaBY =
            transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
          sgn = (točkaBX - točkaAX) * normalX + (pointBY - točkaAY) * normalY;
          če (s < 0,0) {
            this.m_axis.NegativeSelf();
          }
        }
      }
    };
    b2SeparationFunction.prototype.Evaluate = funkcija (transformA, transformB) {
      var os A;
      var axisB;
      var localPointA;
      var localPointB;
      var pointA;
      var pointB;
      var seperation = 0;
      var normalno;
      stikalo (this.m_type) {
        case b2SeparationFunction.e_points: {
          osA = b2Math.MulTMV(transformA.R, this.m_os);
          osB = b2Math.MulTMV(transformB.R, this.m_axis.GetNegative());
          localPointA = this.m_proxyA.GetSupportVertex(axisA);
          localPointB = this.m_proxyB.GetSupportVertex(axisB);
          pointA = b2Math.MulX(transformA, localPointA);
          pointB = b2Math.MulX(transformB, localPointB);
          ločitev =
            (točkaB.x - točkaA.x) * ta.m_os.x +
            (točkaB.y - točkaA.y) * ta.m_os.y;
          povratna ločitev;
        }
        case b2SeparationFunction.e_faceA: {
          normal = b2Math.MulMV(transformA.R, this.m_axis);
          pointA = b2Math.MulX(transformA, this.m_localPoint);
          osB = b2Math.MulTMV(transformB.R, normal.GetNegative());
          localPointB = this.m_proxyB.GetSupportVertex(axisB);
          pointB = b2Math.MulX(transformB, localPointB);
          ločitev =
            (točkaB.x - točkaA.x) * normal.x + (točkaB.y - točkaA.y) * normal.y;
          povratna ločitev;
        }
        case b2SeparationFunction.e_faceB: {
          normal = b2Math.MulMV(transformB.R, this.m_axis);
          pointB = b2Math.MulX(transformB, this.m_localPoint);
          osA = b2Math.MulTMV(transformA.R, normal.GetNegative());
          localPointA = this.m_proxyA.GetSupportVertex(axisA);
          pointA = b2Math.MulX(transformA, localPointA);
          ločitev =
            (točkaA.x - točkaB.x) * normal.x + (točkaA.y - točkaB.y) * normal.y;
          povratna ločitev;
        }
        privzeto:
          b2Settings.b2Assert(false);
          vrnitev 0,0;
      }
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Collision.b2SeparationFunction.e_points = 0x01;
      Box2D.Collision.b2SeparationFunction.e_faceA = 0x02;
      Box2D.Collision.b2SeparationFunction.e_faceB = 0x04;
    });
    b2Simplex.b2Simplex = funkcija () {
      this.m_v1 = novo b2SimplexVertex();
      this.m_v2 = novo b2SimplexVertex();
      this.m_v3 = novo b2SimplexVertex();
      this.m_vertices = nov vektor(3);
    };
    b2Simplex.prototype.b2Simplex = funkcija () {
      this.m_vertices[0] = this.m_v1;
      this.m_vertices[1] = this.m_v2;
      this.m_vertices[2] = this.m_v3;
    };
    b2Simplex.prototype.ReadCache = funkcija (
      predpomnilnik,
      proxyA,
      transformA,
      proxyB,
      transformacijaB
    ) {
      b2Settings.b2Assert(0 <= cache.count && cache.count <= 3);
      var wALocal;
      var wBLocal;
      this.m_count = cache.count;
      var vertices = this.m_vertices;
      for (var i = 0; i < this.m_count; i++) {
        var v = vozlišča[i];
        v.indexA = cache.indexA[i];
        v.indexB = cache.indexB[i];
        wALocal = proxyA.GetVertex(v.indexA);
        wBLocal = proxyB.GetVertex(v.indexB);
        v.wA = b2Math.MulX(transformA, wALocal);
        v.wB = b2Math.MulX(transformB, wBLocal);
        vw = b2Math.SubtractVV(v.wB, v.wA);
        va = 0;
      }
      if (this.m_count > 1) {
        var metric1 = cache.metric;
        var metric2 = this.GetMetric();
        če (
          metric2 < 0,5 * metric1 ||
          2,0 * metrika1 < metrika2 ||
          metrika2 < število.MIN_VALUE
        ) {
          this.m_count = 0;
        }
      }
      if (this.m_count == 0) {
        v = vozlišča [0];
        v.indexA = 0;
        v.indexB = 0;
        wALocal = proxyA.GetVertex(0);
        wBLocal = proxyB.GetVertex(0);
        v.wA = b2Math.MulX(transformA, wALocal);
        v.wB = b2Math.MulX(transformB, wBLocal);
        vw = b2Math.SubtractVV(v.wB, v.wA);
        this.m_count = 1;
      }
    };
    b2Simplex.prototype.WriteCache = funkcija (predpomnilnik) {
      cache.metric = this.GetMetric();
      cache.count = Box2D.parseUInt(this.m_count);
      var vertices = this.m_vertices;
      for (var i = 0; i < this.m_count; i++) {
        cache.indexA[i] = Box2D.parseUInt(vozlišča[i].indexA);
        cache.indexB[i] = Box2D.parseUInt(vozlišča[i].indexB);
      }
    };
    b2Simplex.prototype.GetSearchDirection = funkcija () {
      stikalo (this.m_count) {
        primer 1:
          vrni this.m_v1.w.GetNegative();
        primer 2: {
          var e12 = b2Math.SubtractVV(this.m_v2.w, this.m_v1.w);
          var sgn = b2Math.CrossVV(e12, this.m_v1.w.GetNegative());
          če (sgn > 0,0) {
            vrni b2Math.CrossFV(1.0, e12);
          } drugače {
            vrni b2Math.CrossVF(e12, 1.0);
          }
        }
        privzeto:
          b2Settings.b2Assert(false);
          vrni novo b2Vec2();
      }
    };
    b2Simplex.prototype.GetClosestPoint = funkcija () {
      stikalo (this.m_count) {
        primer 0:
          b2Settings.b2Assert(false);
          vrni novo b2Vec2();
        primer 1:
          vrni this.m_v1.w;
        primer 2:
          vrni novo b2Vec2(
            this.m_v1.a * this.m_v1.wx + this.m_v2.a * this.m_v2.wx,
            this.m_v1.a * this.m_v1.wy + this.m_v2.a * this.m_v2.wy
          );
        privzeto:
          b2Settings.b2Assert(false);
          vrni novo b2Vec2();
      }
    };
    b2Simplex.prototype.GetWitnessPoints = funkcija (pA, pB) {
      stikalo (this.m_count) {
        primer 0:
          b2Settings.b2Assert(false);
          odmor;
        primer 1:
          pA.SetV(this.m_v1.wA);
          pB.SetV(this.m_v1.wB);
          odmor;
        primer 2:
          pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
          pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
          pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
          pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
          odmor;
        primer 3:
          pB.x = pA.x =
            this.m_v1.a * this.m_v1.wA.x +
            this.m_v2.a * this.m_v2.wA.x +
            this.m_v3.a * this.m_v3.wA.x;
          pB.y = pA.y =
            this.m_v1.a * this.m_v1.wA.y +
            this.m_v2.a * this.m_v2.wA.y +
            this.m_v3.a * this.m_v3.wA.y;
          odmor;
        privzeto:
          b2Settings.b2Assert(false);
          odmor;
      }
    };
    b2Simplex.prototype.GetMetric = funkcija () {
      stikalo (this.m_count) {
        primer 0:
          b2Settings.b2Assert(false);
          vrnitev 0,0;
        primer 1:
          vrnitev 0,0;
        primer 2:
          return b2Math.SubtractVV(this.m_v1.w, this.m_v2.w).Dolžina();
        primer 3:
          return b2Math.CrossVV(
            b2Math.SubtractVV(this.m_v2.w, this.m_v1.w),
            b2Math.SubtractVV(this.m_v3.w, this.m_v1.w)
          );
        privzeto:
          b2Settings.b2Assert(false);
          vrnitev 0,0;
      }
    };
    b2Simplex.prototype.Solve2 = funkcija () {
      var w1 = this.m_v1.w;
      var w2 = this.m_v2.w;
      var e12 = b2Math.SubtractVV(w2, w1);
      var d12_2 = -(w1.x * e12.x + w1.y * e12.y);
      če (d12_2 <= 0,0) {
        this.m_v1.a = 1,0;
        this.m_count = 1;
        vrnitev;
      }
      var d12_1 = w2.x * e12.x + w2.y * e12.y;
      če (d12_1 <= 0,0) {
        this.m_v2.a = 1,0;
        this.m_count = 1;
        this.m_v1.Set(this.m_v2);
        vrnitev;
      }
      var inv_d12 = 1,0 / (d12_1 + d12_2);
      this.m_v1.a = d12_1 * inv_d12;
      this.m_v2.a = d12_2 * inv_d12;
      this.m_count = 2;
    };
    b2Simplex.prototype.Solve3 = funkcija () {
      var w1 = this.m_v1.w;
      var w2 = this.m_v2.w;
      var w3 = this.m_v3.w;
      var e12 = b2Math.SubtractVV(w2, w1);
      var w1e12 = b2Math.Dot(w1, e12);
      var w2e12 = b2Math.Dot(w2, e12);
      var d12_1 = w2e12;
      var d12_2 = -w1e12;
      var e13 = b2Math.SubtractVV(w3, w1);
      var w1e13 = b2Math.Dot(w1, e13);
      var w3e13 = b2Math.Dot(w3, e13);
      var d13_1 = w3e13;
      var d13_2 = -w1e13;
      var e23 = b2Math.SubtractVV(w3, w2);
      var w2e23 = b2Math.Dot(w2, e23);
      var w3e23 = b2Math.Dot(w3, e23);
      var d23_1 = w3e23;
      var d23_2 = -w2e23;
      var n123 = b2Math.CrossVV(e12, e13);
      var d123_1 = n123 * b2Math.CrossVV(w2, w3);
      var d123_2 = n123 * b2Math.CrossVV(w3, w1);
      var d123_3 = n123 * b2Math.CrossVV(w1, w2);
      če (d12_2 <= 0,0 && d13_2 <= 0,0) {
        this.m_v1.a = 1,0;
        this.m_count = 1;
        vrnitev;
      }
      če (d12_1 > 0,0 && d12_2 > 0,0 && d123_3 <= 0,0) {
        var inv_d12 = 1,0 / (d12_1 + d12_2);
        this.m_v1.a = d12_1 * inv_d12;
        this.m_v2.a = d12_2 * inv_d12;
        this.m_count = 2;
        vrnitev;
      }
      če (d13_1 > 0,0 && d13_2 > 0,0 && d123_2 <= 0,0) {
        var inv_d13 = 1,0 / (d13_1 + d13_2);
        this.m_v1.a = d13_1 * inv_d13;
        this.m_v3.a = d13_2 * inv_d13;
        this.m_count = 2;
        this.m_v2.Set(this.m_v3);
        vrnitev;
      }
      če (d12_1 <= 0,0 && d23_2 <= 0,0) {
        this.m_v2.a = 1,0;
        this.m_count = 1;
        this.m_v1.Set(this.m_v2);
        vrnitev;
      }
      če (d13_1 <= 0,0 && d23_1 <= 0,0) {
        this.m_v3.a = 1,0;
        this.m_count = 1;
        this.m_v1.Set(this.m_v3);
        vrnitev;
      }
      če (d23_1 > 0,0 && d23_2 > 0,0 && d123_1 <= 0,0) {
        var inv_d23 = 1,0 / (d23_1 + d23_2);
        this.m_v2.a = d23_1 * inv_d23;
        this.m_v3.a = d23_2 * inv_d23;
        this.m_count = 2;
        this.m_v1.Set(this.m_v3);
        vrnitev;
      }
      var inv_d123 = 1,0 / (d123_1 + d123_2 + d123_3);
      this.m_v1.a = d123_1 * inv_d123;
      this.m_v2.a = d123_2 * inv_d123;
      this.m_v3.a = d123_3 * inv_d123;
      this.m_count = 3;
    };
    b2SimplexCache.b2SimplexCache = funkcija () {
      this.indexA = new Vector_a2j_Number(3);
      this.indexB = new Vector_a2j_Number(3);
    };
    b2SimplexVertex.b2SimplexVertex = funkcija () {};
    b2SimplexVertex.prototype.Set = funkcija (drugo) {
      this.wA.SetV(other.wA);
      this.wB.SetV(other.wB);
      this.w.SetV(other.w);
      to.a = drugo.a;
      this.indexA = other.indexA;
      this.indexB = other.indexB;
    };
    b2TimeOfImpact.b2TimeOfImpact = funkcija () {};
    b2TimeOfImpact.TimeOfImpact = funkcija (vnos) {
      ++b2TimeOfImpact.b2_toiCalls;
      var proxyA = input.proxyA;
      var proxyB = input.proxyB;
      var sweepA = input.sweepA;
      var sweepB = input.sweepB;
      b2Settings.b2Assert(sweepA.t0 == sweepB.t0);
      b2Settings.b2Assert(1.0 - sweepA.t0 > Number.MIN_VALUE);
      var radius = proxyA.m_radius + proxyB.m_radius;
      var tolerance = input.tolerance;
      var alfa = 0,0;
      var k_maxIterations = 1000;
      var iter = 0;
      var target = 0,0;
      b2TimeOfImpact.s_cache.count = 0;
      b2TimeOfImpact.s_distanceInput.useRadii = false;
      za (;;) {
        sweepA.GetTransform(b2TimeOfImpact.s_xfA, alfa);
        sweepB.GetTransform(b2TimeOfImpact.s_xfB, alfa);
        b2TimeOfImpact.s_distanceInput.proxyA = proxyA;
        b2TimeOfImpact.s_distanceInput.proxyB = proxyB;
        b2TimeOfImpact.s_distanceInput.transformA = b2TimeOfImpact.s_xfA;
        b2TimeOfImpact.s_distanceInput.transformB = b2TimeOfImpact.s_xfB;
        b2Distance.Distance(
          b2TimeOfImpact.s_distanceOutput,
          b2TimeOfImpact.s_cache,
          b2TimeOfImpact.s_distanceInput
        );
        if (b2TimeOfImpact.s_distanceOutput.distance <= 0,0) {
          alfa = 1,0;
          odmor;
        }
        b2TimeOfImpact.s_fcn.Initialize(
          b2TimeOfImpact.s_cache,
          proxyA,
          b2TimeOfImpact.s_xfA,
          proxyB,
          b2TimeOfImpact.s_xfB
        );
        var separation = b2TimeOfImpact.s_fcn.Evaluate(
          b2TimeOfImpact.s_xfA,
          b2TimeOfImpact.s_xfB
        );
        if (ločitev <= 0,0) {
          alfa = 1,0;
          odmor;
        }
        if (iter == 0) {
          če (ločevanje > polmer) {
            cilj = b2Math.Max(polmer - toleranca, 0,75 * polmer);
          } drugače {
            target = b2Math.Max(ločevanje - toleranca, 0,02 * polmer);
          }
        }
        if (ločevanje - cilj < 0,5 * toleranca) {
          if (iter == 0) {
            alfa = 1,0;
            odmor;
          }
          odmor;
        }
        var newAlpha = alfa;
        {
          var x1 = alfa;
          var x2 = 1,0;
          var f1 = ločitev;
          sweepA.GetTransform(b2TimeOfImpact.s_xfA, x2);
          sweepB.GetTransform(b2TimeOfImpact.s_xfB, x2);
          var f2 = b2TimeOfImpact.s_fcn.Evaluate(
            b2TimeOfImpact.s_xfA,
            b2TimeOfImpact.s_xfB
          );
          če (f2 >= cilj) {
            alfa = 1,0;
            odmor;
          }
          var rootIterCount = 0;
          za (;;) {
            var x = 0;
            if (rootIterCount & 1) {
              x = x1 + ((cilj - f1) * (x2 - x1)) / (f2 - f1);
            } drugače {
              x = 0,5 * (x1 + x2);
            }
            sweepA.GetTransform(b2TimeOfImpact.s_xfA, x);
            sweepB.GetTransform(b2TimeOfImpact.s_xfB, x);
            var f = b2TimeOfImpact.s_fcn.Evaluate(
              b2TimeOfImpact.s_xfA,
              b2TimeOfImpact.s_xfB
            );
            if (b2Math.Abs(f - cilj) < 0,025 * toleranca) {
              novaAlfa = x;
              odmor;
            }
            če (f > cilj) {
              x1 = x;
              f1 = f;
            } drugače {
              x2 = x;
              f2 = f;
            }
            ++rootIterCount;
            ++b2TimeOfImpact.b2_toiRootIters;
            if (rootIterCount == 50) {
              odmor;
            }
          }
          b2TimeOfImpact.b2_toiMaxRootIters = b2Math.Max(
            b2TimeOfImpact.b2_toiMaxRootIters,
            rootIterCount
          );
        }
        if (newAlpha < (1,0 + 100,0 * Number.MIN_VALUE) * alpha) {
          odmor;
        }
        alfa = novaAlfa;
        iter++;
        ++b2TimeOfImpact.b2_toiIters;
        if (iter == k_maxIterations) {
          odmor;
        }
      }
      b2TimeOfImpact.b2_toiMaxIters = b2Math.Max(
        b2TimeOfImpact.b2_toiMaxIters,
        iter
      );
      povratna alfa;
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Collision.b2TimeOfImpact.b2_toiCalls = 0;
      Box2D.Collision.b2TimeOfImpact.b2_toiIters = 0;
      Box2D.Collision.b2TimeOfImpact.b2_toiMaxIters = 0;
      Box2D.Collision.b2TimeOfImpact.b2_toiRootIters = 0;
      Box2D.Collision.b2TimeOfImpact.b2_toiMaxRootIters = 0;
      Box2D.Collision.b2TimeOfImpact.s_cache = novo b2SimplexCache();
      Box2D.Collision.b2TimeOfImpact.s_distanceInput = novo b2DistanceInput();
      Box2D.Collision.b2TimeOfImpact.s_xfA = novo b2Transform();
      Box2D.Collision.b2TimeOfImpact.s_xfB = novo b2Transform();
      Box2D.Collision.b2TimeOfImpact.s_fcn = nova b2SeparationFunction();
      Box2D.Collision.b2TimeOfImpact.s_distanceOutput = novo b2DistanceOutput();
    });
    b2TOIInput.b2TOIInput = funkcija () {
      this.proxyA = novo b2DistanceProxy();
      this.proxyB = novo b2DistanceProxy();
      this.sweepA = novo b2Sweep();
      this.sweepB = novo b2Sweep();
    };
    b2WorldManifold.b2WorldManifold = funkcija () {
      this.m_normal = novo b2Vec2();
    };
    b2WorldManifold.prototype.b2WorldManifold = funkcija () {
      this.m_points = nov vektor(b2Settings.b2_maxManifoldPoints);
      for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
        this.m_points[i] = new b2Vec2();
      }
    };
    b2WorldManifold.prototype.Initialize = funkcija (
      razdelilnik,
      xfA,
      radij A,
      xfB,
      polmerB
    ) {
      if (radiusA === nedefinirano) radiusA = 0;
      if (radiusB === nedefinirano) radiusB = 0;
      if (manifold.m_pointCount == 0) {
        vrnitev;
      }
      var i = 0;
      var tVec;
      var tMat;
      var normalX = 0;
      var normalnoY = 0;
      var planePointX = 0;
      var planePointY = 0;
      var clipPointX = 0;
      var clipPointY = 0;
      stikalo (manifold.m_type) {
        case b2Manifold.e_circles:
          {
            tMat = xfA.R;
            tVec = manifold.m_localPoint;
            var pointAX =
              xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            var pointAY =
              xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tMat = xfB.R;
            tVec = manifold.m_points[0].m_localPoint;
            var pointBX =
              xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            var pointBY =
              xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            var dX = točkaBX - točkaAX;
            var dY = točkaBY - točkaAY;
            var d2 = dX * dX + dY * dY;
            če (d2 > število.MIN_VALUE * število.MIN_VALUE) {
              var d = Math.sqrt(d2);
              this.m_normal.x = dX / d;
              this.m_normal.y = dY / d;
            } drugače {
              to.m_normalno.x = 1;
              this.m_normal.y = 0;
            }
            var cAX = točkaAX + polmerA * this.m_normal.x;
            var cAY = pointAY + polmerA * this.m_normal.y;
            var cBX = točkaBX - polmerB * this.m_normal.x;
            var cBY = pointBY - polmerB * this.m_normal.y;
            this.m_points[0].x = 0,5 * (cAX + cBX);
            this.m_points[0].y = 0,5 * (cAY + cBY);
          }
          odmor;
        primer b2Manifold.e_faceA:
          {
            tMat = xfA.R;
            tVec = manifold.m_localPlaneNormal;
            normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tMat = xfA.R;
            tVec = manifold.m_localPoint;
            ravninaPointX =
              xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            ravninaPointY =
              xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            this.m_normal.x = normalX;
            this.m_normal.y = normalnoY;
            for (i = 0; i < manifold.m_pointCount; i++) {
              tMat = xfB.R;
              tVec = manifold.m_points[i].m_localPoint;
              clipPointX =
                xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
              clipPointY =
                xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
              to.m_točk[i].x =
                clipPointX +
                0,5 *
                  (polmer A -
                    (clipPointX - planePointX) * normalX -
                    (clipPointY - planePointY) * normalY -
                    polmerB) *
                  normalX;
              to.m_točk[i].y =
                clipPointY +
                0,5 *
                  (polmer A -
                    (clipPointX - planePointX) * normalX -
                    (clipPointY - planePointY) * normalY -
                    polmerB) *
                  normalnoY;
            }
          }
          odmor;
        primer b2Manifold.e_faceB:
          {
            tMat = xfB.R;
            tVec = manifold.m_localPlaneNormal;
            normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tMat = xfB.R;
            tVec = manifold.m_localPoint;
            ravninaPointX =
              xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            ravninaPointY =
              xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            this.m_normal.x = -normalX;
            this.m_normal.y = -normalY;
            for (i = 0; i < manifold.m_pointCount; i++) {
              tMat = xfA.R;
              tVec = manifold.m_points[i].m_localPoint;
              clipPointX =
                xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
              clipPointY =
                xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
              to.m_točk[i].x =
                clipPointX +
                0,5 *
                  (polmer B -
                    (clipPointX - planePointX) * normalX -
                    (clipPointY - planePointY) * normalY -
                    polmerA) *
                  normalX;
              to.m_točk[i].y =
                clipPointY +
                0,5 *
                  (polmer B -
                    (clipPointX - planePointX) * normalX -
                    (clipPointY - planePointY) * normalY -
                    polmerA) *
                  normalnoY;
            }
          }
          odmor;
      }
    };
    ClipVertex.ClipVertex = funkcija () {
      this.v = novo b2Vec2();
      this.id = new b2ContactID();
    };
    ClipVertex.prototype.Set = funkcija (drugo) {
      this.v.SetV(other.v);
      this.id.Set(other.id);
    };
    Features.Features = funkcija () {};
    Object.defineProperty(Features.prototype, "referenceEdge", {
      enumerable: false,
      nastavljivo: res,
      get: funkcija () {
        vrni to._referenceEdge;
      },
    });
    Object.defineProperty(Features.prototype, "referenceEdge", {
      enumerable: false,
      nastavljivo: res,
      set: funkcija (vrednost) {
        če (vrednost === nedefinirano) vrednost = 0;
        this._referenceEdge = vrednost;
        this._m_id._key =
          (ta._m_id._ključ & 0xffffff00) | (this._referenceEdge & 0x000000ff);
      },
    });
    Object.defineProperty(Features.prototype, "incidentEdge", {
      enumerable: false,
      nastavljivo: res,
      get: funkcija () {
        vrni to._incidentEdge;
      },
    });
    Object.defineProperty(Features.prototype, "incidentEdge", {
      enumerable: false,
      nastavljivo: res,
      set: funkcija (vrednost) {
        če (vrednost === nedefinirano) vrednost = 0;
        this._incidentEdge = vrednost;
        this._m_id._key =
          (ta._m_id._ključ & 0xffff00ff) |
          ((this._incidentEdge << 8) & 0x0000ff00);
      },
    });
    Object.defineProperty(Features.prototype, "incidentVertex", {
      enumerable: false,
      nastavljivo: res,
      get: funkcija () {
        vrni to._incidentVertex;
      },
    });
    Object.defineProperty(Features.prototype, "incidentVertex", {
      enumerable: false,
      nastavljivo: res,
      set: funkcija (vrednost) {
        če (vrednost === nedefinirano) vrednost = 0;
        this._incidentVertex = vrednost;
        this._m_id._key =
          (ta._m_id._ključ & 0xff00ffff) |
          ((this._incidentVertex << 16) & 0x00ff0000);
      },
    });
    Object.defineProperty(Features.prototype, "flip", {
      enumerable: false,
      nastavljivo: res,
      get: funkcija () {
        vrni to._flip;
      },
    });
    Object.defineProperty(Features.prototype, "flip", {
      enumerable: false,
      nastavljivo: res,
      set: funkcija (vrednost) {
        če (vrednost === nedefinirano) vrednost = 0;
        this._flip = vrednost;
        this._m_id._key =
          (ta._m_id._ključ & 0x00ffffff) | ((this._flip << 24) & 0xff000000);
      },
    });
  })();
  (funkcija () {
    var b2Color = Box2D.Common.b2Color,
      b2notranji = Box2D.Common.b2notranji,
      b2Settings = Box2D.Common.b2Settings,
      b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
      b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef,
      b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape,
      b2MassData = Box2D.Collision.Shapes.b2MassData,
      b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
      b2Shape = Box2D.Collision.Shapes.b2Shape,
      b2Mat22 = Box2D.Common.Math.b2Mat22,
      b2Mat33 = Box2D.Common.Math.b2Mat33,
      b2Math = Box2D.Common.Math.b2Math,
      b2Sweep = Box2D.Common.Math.b2Sweep,
      b2Transform = Box2D.Common.Math.b2Transform,
      b2Vec2 = Box2D.Common.Math.b2Vec2,
      b2Vec3 = Box2D.Common.Math.b2Vec3,
      b2Body = Box2D.Dynamics.b2Body,
      b2BodyDef = Box2D.Dynamics.b2BodyDef,
      b2ContactFilter = Box2D.Dynamics.b2ContactFilter,
      b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse,
      b2ContactListener = Box2D.Dynamics.b2ContactListener,
      b2ContactManager = Box2D.Dynamics.b2ContactManager,
      b2DebugDraw = Box2D.Dynamics.b2DebugDraw,
      b2DestructionListener = Box2D.Dynamics.b2DestructionListener,
      b2FilterData = Box2D.Dynamics.b2FilterData,
      b2Fixture = Box2D.Dynamics.b2Fixture,
      b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
      b2Island = Box2D.Dynamics.b2Island,
      b2TimeStep = Box2D.Dynamics.b2TimeStep,
      b2World = Box2D.Dynamics.b2World,
      b2AABB = Box2D.Collision.b2AABB,
      b2Bound = Box2D.Collision.b2Bound,
      b2BoundValues ​​= Box2D.Collision.b2BoundValues,
      b2Collision = Box2D.Collision.b2Collision,
      b2ContactID = Box2D.Collision.b2ContactID,
      b2ContactPoint = Box2D.Collision.b2ContactPoint,
      b2Distance = Box2D.Collision.b2Distance,
      b2DistanceInput = Box2D.Collision.b2DistanceInput,
      b2DistanceOutput = Box2D.Collision.b2DistanceOutput,
      b2DistanceProxy = Box2D.Collision.b2DistanceProxy,
      b2DynamicTree = Box2D.Collision.b2DynamicTree,
      b2DynamicTreeBroadPhase = Box2D.Collision.b2DynamicTreeBroadPhase,
      b2DynamicTreeNode = Box2D.Collision.b2DynamicTreeNode,
      b2DynamicTreePair = Box2D.Collision.b2DynamicTreePair,
      b2Manifold = Box2D.Collision.b2Manifold,
      b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint,
      b2Point = Box2D.Collision.b2Point,
      b2RayCastInput = Box2D.Collision.b2RayCastInput,
      b2RayCastOutput = Box2D.Collision.b2RayCastOutput,
      b2Segment = Box2D.Collision.b2Segment,
      b2SeparationFunction = Box2D.Collision.b2SeparationFunction,
      b2Simplex = Box2D.Collision.b2Simplex,
      b2SimplexCache = Box2D.Collision.b2SimplexCache,
      b2SimplexVertex = Box2D.Collision.b2SimplexVertex,
      b2TimeOfImpact = Box2D.Collision.b2TimeOfImpact,
      b2TOIInput = Box2D.Collision.b2TOIInput,
      b2WorldManifold = Box2D.Collision.b2WorldManifold,
      ClipVertex = Box2D.Collision.ClipVertex,
      Lastnosti = Box2D.Collision.Features,
      IBroadPhase = Box2D.Collision.IBroadPhase;

    Box2D.inherit(b2CircleShape, Box2D.Collision.Shapes.b2Shape);
    b2CircleShape.prototype.__super = Box2D.Collision.Shapes.b2Shape.prototype;
    b2CircleShape.b2CircleShape = funkcija () {
      Box2D.Collision.Shapes.b2Shape.b2Shape.apply(to, argumenti);
      this.m_p = novo b2Vec2();
    };
    b2CircleShape.prototype.Copy = funkcija () {
      var s = new b2CircleShape();
      s.Set(to);
      vrni s;
    };
    b2CircleShape.prototype.Set = funkcija (drugo) {
      this.__super.Set.call(this, other);
      if (Box2D.is(other, b2CircleShape)) {
        var other2 = drug primerek b2CircleShape? drugo : nič;
        this.m_p.SetV(other2.m_p);
      }
    };
    b2CircleShape.prototype.TestPoint = funkcija (transformacija, p) {
      var tMat = transform.R;
      var dX =
        transform.položaj.x +
        (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
      var dY =
        transform.position.y +
        (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
      dX = px - dX;
      dY = py - dY;
      vrni dX * dX + dY * dY <= this.m_radius * this.m_radius;
    };
    b2CircleShape.prototype.RayCast = funkcija (izhod, vnos, transformacija) {
      var tMat = transform.R;
      var positionX =
        transform.položaj.x +
        (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
      var positionY =
        transform.position.y +
        (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
      var sX = input.p1.x - položajX;
      var sY = input.p1.y - položajY;
      var b = sX * sX + sY * sY - this.m_radius * this.m_radius;
      var rX = input.p2.x - input.p1.x;
      var rY = input.p2.y - input.p1.y;
      var c = sX * rX + sY * rY;
      var rr = rX * rX + rY * rY;
      var sigma = c * c - rr * b;
      if (sigma < 0,0 || rr < Number.MIN_VALUE) {
        vrni false;
      }
      var a = -(c + Math.sqrt(sigma));
      if (0.0 <= a && a <= input.maxFraction * rr) {
        a /= rr;
        output.fraction = a;
        output.normal.x = sX + a * rX;
        output.normal.y = sY + a * rY;
        output.normal.Normalize();
        vrni resnico;
      }
      vrni false;
    };
    b2CircleShape.prototype.ComputeAABB = funkcija (aabb, transformacija) {
      var tMat = transform.R;
      var pX =
        transform.položaj.x +
        (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
      var pY =
        transform.position.y +
        (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
      aabb.lowerBound.Set(pX - this.m_radius, pY - this.m_radius);
      aabb.upperBound.Set(pX + this.m_radius, pY + this.m_radius);
    };
    b2CircleShape.prototype.ComputeMass = funkcija (massData, density) {
      if (gostota === nedefinirano) gostota = 0;
      massData.mass = gostota * b2Settings.b2_pi * this.m_radius * this.m_radius;
      massData.center.SetV(this.m_p);
      masovniPodatki.I =
        massData.mass *
        (0,5 * this.m_radius * this.m_radius +
          (ta.m_p.x * ta.m_p.x + ta.m_p.y * ta.m_p.y));
    };
    b2CircleShape.prototype.ComputeSubmergedArea = funkcija (
      normalno,
      odmik,
      xf,
      c
    ) {
      če (odmik === nedefiniran) odmik = 0;
      var p = b2Math.MulX(xf, this.m_p);
      var l = -(b2Math.Dot(normal, p) - odmik);
      if (l < -this.m_radius + Number.MIN_VALUE) {
        vrni 0;
      }
      if (l > this.m_radius) {
        c.SetV(p);
        return Math.PI * this.m_radius * this.m_radius;
      }
      var r2 = this.m_radius * this.m_radius;
      var l2 = l * l;
      var območje =
        r2 * (Math.asin(l / this.m_radius) + Math.PI / 2) +
        l * Math.sqrt(r2 - l2);
      var com = ((-2 / 3) * Math.pow(r2 - l2, 1,5)) / območje;
      cx = px + normal.x * com;
      cy = py + normal.y * com;
      povratno območje;
    };
    b2CircleShape.prototype.GetLocalPosition = funkcija () {
      vrni to.m_p;
    };
    b2CircleShape.prototype.SetLocalPosition = funkcija (položaj) {
      this.m_p.SetV(position);
    };
    b2CircleShape.prototype.GetRadius = funkcija () {
      vrni this.m_radius;
    };
    b2CircleShape.prototype.SetRadius = funkcija (polmer) {
      če (polmer === nedefinirano) radij = 0;
      this.m_radius = polmer;
    };
    b2CircleShape.prototype.b2CircleShape = funkcija (polmer) {
      če (polmer === nedefinirano) radij = 0;
      this.__super.b2Shape.call(this);
      this.m_type = b2Shape.e_circleShape;
      this.m_radius = polmer;
    };
    b2EdgeChainDef.b2EdgeChainDef = funkcija () {};
    b2EdgeChainDef.prototype.b2EdgeChainDef = funkcija () {
      this.vertexCount = 0;
      this.isALoop = res;
      this.vertices = [];
    };
    Box2D.inherit(b2EdgeShape, Box2D.Collision.Shapes.b2Shape);
    b2EdgeShape.prototype.__super = Box2D.Collision.Shapes.b2Shape.prototype;
    b2EdgeShape.b2EdgeShape = funkcija () {
      Box2D.Collision.Shapes.b2Shape.b2Shape.apply(to, argumenti);
      this.s_supportVec = novo b2Vec2();
      this.m_v1 = novo b2Vec2();
      this.m_v2 = novo b2Vec2();
      this.m_coreV1 = novo b2Vec2();
      this.m_coreV2 = novo b2Vec2();
      this.m_normal = novo b2Vec2();
      this.m_direction = novo b2Vec2();
      this.m_cornerDir1 = novo b2Vec2();
      this.m_cornerDir2 = novo b2Vec2();
    };
    b2EdgeShape.prototype.TestPoint = funkcija (pretvorba, p) {
      vrni false;
    };
    b2EdgeShape.prototype.RayCast = funkcija (izhod, vnos, transformacija) {
      var tMat;
      var rX = input.p2.x - input.p1.x;
      var rY = input.p2.y - input.p1.y;
      tMat = transform.R;
      var v1X =
        transform.položaj.x +
        (tMat.col1.x * this.m_v1.x + tMat.col2.x * this.m_v1.y);
      var v1Y =
        transform.position.y +
        (tMat.col1.y * this.m_v1.x + tMat.col2.y * this.m_v1.y);
      var nX =
        transform.position.y +
        (tMat.col1.y * this.m_v2.x + tMat.col2.y * this.m_v2.y) -
        v1Y;
      var nY = -(
        transform.položaj.x +
        (tMat.col1.x * this.m_v2.x + tMat.col2.x * this.m_v2.y) -
        v1X
      );
      var k_slop = 100,0 * Število.MIN_VREDNOST;
      var denom = -(rX * nX + rY * nY);
      if (denom > k_slop) {
        var bX = input.p1.x - v1X;
        var bY = input.p1.y - v1Y;
        var a = bX * nX + bY * nY;
        if (0.0 <= a && a <= input.maxFraction * denom) {
          var mu2 = -rX * bY + rY * bX;
          if (-k_slop * denom <= mu2 && mu2 <= denom * (1,0 + k_slop)) {
            a /= denom;
            output.fraction = a;
            var nLen = Math.sqrt(nX * nX + nY * nY);
            izhod.normal.x = nX / nLen;
            izhod.normal.y = nY / nLen;
            vrni resnico;
          }
        }
      }
      vrni false;
    };
    b2EdgeShape.prototype.ComputeAABB = funkcija (aabb, transformacija) {
      var tMat = transform.R;
      var v1X =
        transform.položaj.x +
        (tMat.col1.x * this.m_v1.x + tMat.col2.x * this.m_v1.y);
      var v1Y =
        transform.position.y +
        (tMat.col1.y * this.m_v1.x + tMat.col2.y * this.m_v1.y);
      var v2X =
        transform.položaj.x +
        (tMat.col1.x * this.m_v2.x + tMat.col2.x * this.m_v2.y);
      var v2Y =
        transform.position.y +
        (tMat.col1.y * this.m_v2.x + tMat.col2.y * this.m_v2.y);
      če (v1X < v2X) {
        aabb.lowerBound.x = v1X;
        aabb.upperBound.x = v2X;
      } drugače {
        aabb.lowerBound.x = v2X;
        aabb.upperBound.x = v1X;
      }
      if (v1Y < v2Y) {
        aabb.lowerBound.y = v1Y;
        aabb.upperBound.y = v2Y;
      } drugače {
        aabb.lowerBound.y = v2Y;
        aabb.upperBound.y = v1Y;
      }
    };
    b2EdgeShape.prototype.ComputeMass = funkcija (massData, density) {
      if (gostota === nedefinirano) gostota = 0;
      massData.mass = 0;
      massData.center.SetV(this.m_v1);
      masiPodatki.I = 0;
    };
    b2EdgeShape.prototype.ComputeSubmergedArea = funkcija (
      normalno,
      odmik,
      xf,
      c
    ) {
      če (odmik === nedefiniran) odmik = 0;
      var v0 = novo b2Vec2(normal.x * odmik, normal.y * odmik);
      var v1 = b2Math.MulX(xf, this.m_v1);
      var v2 = b2Math.MulX(xf, this.m_v2);
      var d1 = b2Math.Dot(normal, v1) - odmik;
      var d2 = b2Math.Dot(normal, v2) - odmik;
      če (d1 > 0) {
        if (d2 > 0) {
          vrni 0;
        } drugače {
          v1.x = (-d2 / (d1 - d2)) * v1.x + (d1 / (d1 - d2)) * v2.x;
          v1.y = (-d2 / (d1 - d2)) * v1.y + (d1 / (d1 - d2)) * v2.y;
        }
      } drugače {
        if (d2 > 0) {
          v2.x = (-d2 / (d1 - d2)) * v1.x + (d1 / (d1 - d2)) * v2.x;
          v2.y = (-d2 / (d1 - d2)) * v1.y + (d1 / (d1 - d2)) * v2.y;
        } drugače {
        }
      }
      cx = (v0.x + v1.x + v2.x) / 3;
      cy = (v0.y + v1.y + v2.y) / 3;
      vrnitev (
        0,5 * ((v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x))
      );
    };
    b2EdgeShape.prototype.GetLength = funkcija () {
      vrni this.m_length;
    };
    b2EdgeShape.prototype.GetVertex1 = funkcija () {
      vrni to.m_v1;
    };
    b2EdgeShape.prototype.GetVertex2 = funkcija () {
      vrni to.m_v2;
    };
    b2EdgeShape.prototype.GetCoreVertex1 = funkcija () {
      vrni this.m_coreV1;
    };
    b2EdgeShape.prototype.GetCoreVertex2 = funkcija () {
      vrni this.m_coreV2;
    };
    b2EdgeShape.prototype.GetNormalVector = funkcija () {
      vrni this.m_normal;
    };
    b2EdgeShape.prototype.GetDirectionVector = funkcija () {
      vrni this.m_direction;
    };
    b2EdgeShape.prototype.GetCorner1Vector = funkcija () {
      vrni this.m_cornerDir1;
    };
    b2EdgeShape.prototype.GetCorner2Vector = funkcija () {
      vrni this.m_cornerDir2;
    };
    b2EdgeShape.prototype.Corner1IsConvex = funkcija () {
      vrni this.m_cornerConvex1;
    };
    b2EdgeShape.prototype.Corner2IsConvex = funkcija () {
      vrni this.m_cornerConvex2;
    };
    b2EdgeShape.prototype.GetFirstVertex = funkcija (xf) {
      var tMat = xf.R;
      vrni novo b2Vec2(
        xf.položaj.x +
          (tMat.col1.x * this.m_coreV1.x + tMat.col2.x * this.m_coreV1.y),
        xf.position.y +
          (tMat.col1.y * this.m_coreV1.x + tMat.col2.y * this.m_coreV1.y)
      );
    };
    b2EdgeShape.prototype.GetNextEdge = funkcija () {
      vrni this.m_nextEdge;
    };
    b2EdgeShape.prototype.GetPrevEdge = funkcija () {
      vrni this.m_prevEdge;
    };
    b2EdgeShape.prototype.Support = funkcija (xf, dX, dY) {
      if (dX === nedefinirano) dX = 0;
      if (dY === nedefinirano) dY = 0;
      var tMat = xf.R;
      var v1X =
        xf.položaj.x +
        (tMat.col1.x * this.m_coreV1.x + tMat.col2.x * this.m_coreV1.y);
      var v1Y =
        xf.position.y +
        (tMat.col1.y * this.m_coreV1.x + tMat.col2.y * this.m_coreV1.y);
      var v2X =
        xf.položaj.x +
        (tMat.col1.x * this.m_coreV2.x + tMat.col2.x * this.m_coreV2.y);
      var v2Y =
        xf.position.y +
        (tMat.col1.y * this.m_coreV2.x + tMat.col2.y * this.m_coreV2.y);
      if (v1X * dX + v1Y * dY > v2X * dX + v2Y * dY) {
        this.s_supportVec.x = v1X;
        this.s_supportVec.y = v1Y;
      } drugače {
        this.s_supportVec.x = v2X;
        this.s_supportVec.y = v2Y;
      }
      vrni this.s_supportVec;
    };
    b2EdgeShape.prototype.b2EdgeShape = funkcija (v1, v2) {
      this.__super.b2Shape.call(this);
      this.m_type = b2Shape.e_edgeShape;
      this.m_prevEdge = null;
      this.m_nextEdge = null;
      this.m_v1 = v1;
      this.m_v2 = v2;
      this.m_direction.Set(this.m_v2.x - this.m_v1.x, this.m_v2.y - this.m_v1.y);
      this.m_length = this.m_direction.Normalize();
      this.m_normal.Set(this.m_direction.y, -this.m_direction.x);
      this.m_coreV1.Set(
        -b2Settings.b2_toiSlop * (this.m_normal.x - this.m_direction.x) +
          this.m_v1.x,
        -b2Settings.b2_toiSlop * (this.m_normal.y - this.m_direction.y) +
          ta.m_v1.y
      );
      this.m_coreV2.Set(
        -b2Settings.b2_toiSlop * (this.m_normal.x + this.m_direction.x) +
          this.m_v2.x,
        -b2Settings.b2_toiSlop * (this.m_normal.y + this.m_direction.y) +
          this.m_v2.y
      );
      this.m_cornerDir1 = this.m_normal;
      this.m_cornerDir2.Set(-this.m_normal.x, -this.m_normal.y);
    };
    b2EdgeShape.prototype.SetPrevEdge = funkcija (rob, jedro, cornerDir, konveksno) {
      this.m_prevEdge = rob;
      this.m_coreV1 = jedro;
      this.m_cornerDir1 = kotniDir;
      this.m_cornerConvex1 = konveksen;
    };
    b2EdgeShape.prototype.SetNextEdge = funkcija (rob, jedro, cornerDir, konveksno) {
      this.m_nextEdge = rob;
      this.m_coreV2 = jedro;
      this.m_cornerDir2 = kotniDir;
      this.m_cornerConvex2 = konveksen;
    };
    b2MassData.b2MassData = funkcija () {
      ta.masa = 0,0;
      this.center = novo b2Vec2(0, 0);
      to.I = 0,0;
    };
    Box2D.inherit(b2PolygonShape, Box2D.Collision.Shapes.b2Shape);
    b2PolygonShape.prototype.__super = Box2D.Collision.Shapes.b2Shape.prototype;
    b2PolygonShape.b2PolygonShape = funkcija () {
      Box2D.Collision.Shapes.b2Shape.b2Shape.apply(to, argumenti);
    };
    b2PolygonShape.prototype.Copy = funkcija () {
      var s = new b2PolygonShape();
      s.Set(to);
      vrni s;
    };
    b2PolygonShape.prototype.Set = funkcija (drugo) {
      this.__super.Set.call(this, other);
      if (Box2D.is(other, b2PolygonShape)) {
        var other2 = drug primerek b2PolygonShape? drugo : nič;
        this.m_centroid.SetV(other2.m_centroid);
        this.m_vertexCount = other2.m_vertexCount;
        this.Reserve(this.m_vertexCount);
        for (var i = 0; i < this.m_vertexCount; i++) {
          this.m_vertices[i].SetV(other2.m_vertices[i]);
          this.m_normals[i].SetV(other2.m_normals[i]);
        }
      }
    };
    b2PolygonShape.prototype.SetAsArray = funkcija (vozlišča, število točk) {
      če (Štetje vertex === nedefinirano) Število vertex = 0;
      var v = nov vektor();
      var i = 0,
        tVec;
      for (i = 0; i < vertices.length; ++i) {
        tVec = vozlišča[i];
        v.push(tVec);
      }
      this.SetAsVector(v, vertexCount);
    };
    b2PolygonShape.AsArray = funkcija (točke, število točk) {
      če (Štetje vertex === nedefinirano) Število vertex = 0;
      var polygonShape = new b2PolygonShape();
      polygonShape.SetAsArray(vozlišča, število točk);
      vrni polygonShape;
    };
    b2PolygonShape.prototype.SetAsVector = funkcija (točke, vertexCount) {
      če (Štetje vertex === nedefinirano) Število vertex = 0;
      if (vertexCount == 0) vertexCount = vertices.length;
      b2Settings.b2Assert(2 <= vertexCount);
      this.m_vertexCount = vertexCount;
      this.Reserve(vertexCount);
      var i = 0;
      for (i = 0; i < this.m_vertexCount; i++) {
        this.m_vertices[i].SetV(vertices[i]);
      }
      for (i = 0; i < this.m_vertexCount; ++i) {
        var i1 = parseInt(i);
        var i2 = parseInt(i + 1 < this.m_vertexCount ? i + 1 : 0);
        var edge = b2Math.SubtractVV(this.m_vertices[i2], this.m_vertices[i1]);
        b2Settings.b2Assert(edge.LengthSquared() > Number.MIN_VALUE);
        this.m_normals[i].SetV(b2Math.CrossVF(edge, 1.0));
        this.m_normals[i].Normalize();
      }
      this.m_centroid = b2PolygonShape.ComputeCentroid(
        this.m_vertices,
        this.m_vertexCount
      );
    };
    b2PolygonShape.AsVector = funkcija (točke, število točk) {
      če (Štetje vertex === nedefinirano) Število vertex = 0;
      var polygonShape = new b2PolygonShape();
      polygonShape.SetAsVector(vozlišča, število točk);
      vrni polygonShape;
    };
    b2PolygonShape.prototype.SetAsBox = funkcija (hx, hy) {
      if (hx === nedefinirano) hx = 0;
      if (hy === nedefinirano) hy = 0;
      this.m_vertexCount = 4;
      this.Reserve(4);
      this.m_vertices[0].Set(-hx, -hy);
      this.m_vertices[1].Set(hx, -hy);
      this.m_vertices[2].Set(hx, hy);
      this.m_vertices[3].Set(-hx, hy);
      this.m_normals[0].Set(0.0, -1.0);
      this.m_normals[1].Set(1.0, 0.0);
      this.m_normals[2].Set(0.0, 1.0);
      this.m_normals[3].Set(-1.0, 0.0);
      this.m_centroid.SetZero();
    };
    b2PolygonShape.AsBox = funkcija (hx, hy) {
      if (hx === nedefinirano) hx = 0;
      if (hy === nedefinirano) hy = 0;
      var polygonShape = new b2PolygonShape();
      polygonShape.SetAsBox(hx, hy);
      vrni polygonShape;
    };
    b2PolygonShape.prototype.SetAsOrientedBox = funkcija (hx, hy, center, kot) {
      if (hx === nedefinirano) hx = 0;
      if (hy === nedefinirano) hy = 0;
      if (center === nedefinirano) center = null;
      če (kot === nedefiniran) kot = 0,0;
      this.m_vertexCount = 4;
      this.Reserve(4);
      this.m_vertices[0].Set(-hx, -hy);
      this.m_vertices[1].Set(hx, -hy);
      this.m_vertices[2].Set(hx, hy);
      this.m_vertices[3].Set(-hx, hy);
      this.m_normals[0].Set(0.0, -1.0);
      this.m_normals[1].Set(1.0, 0.0);
      this.m_normals[2].Set(0.0, 1.0);
      this.m_normals[3].Set(-1.0, 0.0);
      this.m_centroid = center;
      var xf = new b2Transform();
      xf.position = center;
      xf.R.Set(kot);
      for (var i = 0; i < this.m_vertexCount; ++i) {
        this.m_vertices[i] = b2Math.MulX(xf, this.m_vertices[i]);
        this.m_normals[i] = b2Math.MulMV(xf.R, this.m_normals[i]);
      }
    };
    b2PolygonShape.AsOrientedBox = funkcija (hx, hy, središče, kot) {
      if (hx === nedefinirano) hx = 0;
      if (hy === nedefinirano) hy = 0;
      if (center === nedefinirano) center = null;
      če (kot === nedefiniran) kot = 0,0;
      var polygonShape = new b2PolygonShape();
      polygonShape.SetAsOrientedBox(hx, hy, središče, kot);
      vrni polygonShape;
    };
    b2PolygonShape.prototype.SetAsEdge = funkcija (v1, v2) {
      this.m_vertexCount = 2;
      this.Reserve(2);
      this.m_vertices[0].SetV(v1);
      this.m_vertices[1].SetV(v2);
      this.m_centroid.x = 0,5 * (v1.x + v2.x);
      this.m_centroid.y = 0,5 * (v1.y + v2.y);
      this.m_normals[0] = b2Math.CrossVF(b2Math.SubtractVV(v2, v1), 1.0);
      this.m_normals[0].Normalize();
      this.m_normals[1].x = -this.m_normals[0].x;
      this.m_normals[1].y = -this.m_normals[0].y;
    };
    b2PolygonShape.AsEdge = funkcija (v1, v2) {
      var polygonShape = new b2PolygonShape();
      polygonShape.SetAsEdge(v1, v2);
      vrni polygonShape;
    };
    b2PolygonShape.prototype.TestPoint = funkcija (xf, p) {
      var tVec;
      var tMat = xf.R;
      var tX = px - xf.position.x;
      var tY = py - xf.position.y;
      var pLocalX = tX * tMat.col1.x + tY * tMat.col1.y;
      var pLocalY = tX * tMat.col2.x + tY * tMat.col2.y;
      for (var i = 0; i < this.m_vertexCount; ++i) {
        tVec = this.m_vertices[i];
        tX = pLocalX - tVec.x;
        tY = pLocalY - tVec.y;
        tVec = this.m_normals[i];
        var dot = tVec.x * tX + tVec.y * tY;
        če (pika > 0,0) {
          vrni false;
        }
      }
      vrni resnico;
    };
    b2PolygonShape.prototype.RayCast = funkcija (izhod, vnos, transformacija) {
      var nižja = 0,0;
      var upper = input.maxFraction;
      var tX = 0;
      var tY = 0;
      var tMat;
      var tVec;
      tX = input.p1.x - transform.position.x;
      tY = input.p1.y - transform.position.y;
      tMat = transform.R;
      var p1X = tX * tMat.col1.x + tY * tMat.col1.y;
      var p1Y = tX * tMat.col2.x + tY * tMat.col2.y;
      tX = input.p2.x - transform.position.x;
      tY = input.p2.y - transform.position.y;
      tMat = transform.R;
      var p2X = tX * tMat.col1.x + tY * tMat.col1.y;
      var p2Y = tX * tMat.col2.x + tY * tMat.col2.y;
      var dX = p2X - p1X;
      var dY = p2Y - p1Y;
      var index = parseInt(-1);
      for (var i = 0; i < this.m_vertexCount; ++i) {
        tVec = this.m_vertices[i];
        tX = tVec.x - p1X;
        tY = tVec.y - p1Y;
        tVec = this.m_normals[i];
        spremenljiv števec = tVec.x * tX + tVec.y * tY;
        spremenljiv imenovalec = tVec.x * dX + tVec.y * dY;
        če (imenovalec == 0,0) {
          če (števec < 0,0) {
            vrni false;
          }
        } drugače {
          če (imenovalec < 0,0 && števec < spodnji * imenovalec) {
            spodnji = števec / imenovalec;
            indeks = i;
          } else if (imenovalec > 0,0 && števec < zgornji * imenovalec) {
            zgornji = števec / imenovalec;
          }
        }
        if (upper < lower - Number.MIN_VALUE) {
          vrni false;
        }
      }
      če (indeks >= 0) {
        output.fraction = nižji;
        tMat = transform.R;
        tVec = this.m_normals[index];
        output.normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        output.normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        vrni resnico;
      }
      vrni false;
    };
    b2PolygonShape.prototype.ComputeAABB = funkcija (aabb, xf) {
      var tMat = xf.R;
      var tVec = this.m_vertices[0];
      var lowerX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      var lowerY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      var zgornji X = spodnji X;
      var zgornji Y = spodnji Y;
      for (var i = 1; i < this.m_vertexCount; ++i) {
        tVec = this.m_vertices[i];
        var vX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
        var vY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        spodnjiX = spodnjiX < vX? spodnji X : vX;
        nižji Y = nižji Y < vY? spodnji Y : vY;
        zgornjiX = zgornjiX > vX? zgornji X : vX;
        zgornjiY = zgornjiY > vY? zgornjiY : vY;
      }
      aabb.lowerBound.x = lowerX - this.m_radius;
      aabb.lowerBound.y = lowerY - this.m_radius;
      aabb.upperBound.x = upperX + this.m_radius;
      aabb.upperBound.y = upperY + this.m_radius;
    };
    b2PolygonShape.prototype.ComputeMass = funkcija (massData, density) {
      if (gostota === nedefinirano) gostota = 0;
      if (this.m_vertexCount == 2) {
        massData.center.x = 0,5 * ( this.m_vertices[0].x + this.m_vertices[1].x);
        massData.center.y = 0,5 * (this.m_vertices[0].y + this.m_vertices[1].y);
        massData.mass = 0,0;
        massData.I = 0,0;
        vrnitev;
      }
      var centerX = 0,0;
      var centerY = 0,0;
      var površina = 0,0;
      var I = 0,0;
      var p1X = 0,0;
      var p1Y = 0,0;
      var k_inv3 = 1,0 / 3,0;
      for (var i = 0; i < this.m_vertexCount; ++i) {
        var p2 = this.m_vertices[i];
        var p3 =
          i + 1 < this.m_vertexCount
            ? this.m_vertices[parseInt(i + 1)]
            : this.m_vertices[0];
        var e1X = p2.x - p1X;
        var e1Y = p2.y - p1Y;
        var e2X = p3.x - p1X;
        var e2Y = p3.y - p1Y;
        var. D = e1X * e2Y - e1Y * e2X;
        var triangleArea = 0,5 * D;
        površina += trikotnikArea;
        centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
        centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);
        var px = p1X;
        var py = p1Y;
        var ex1 = e1X;
        var ey1 = e1Y;
        var ex2 = e2X;
        var ey2 = e2Y;
        var intx2 =
          k_inv3 *
            (0,25 * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px * ex2)) +
          0,5 * px * px;
        var inty2 =
          k_inv3 *
            (0,25 * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py * ey2)) +
          0,5 * py * py;
        I += D * (intx2 + inty2);
      }
      massData.mass = gostota * površina;
      centerX *= 1,0 / območje;
      centerY *= 1,0 / površina;
      massData.center.Set(centerX, centerY);
      massData.I = gostota * I;
    };
    b2PolygonShape.prototype.ComputeSubmergedArea = funkcija (
      normalno,
      odmik,
      xf,
      c
    ) {
      če (odmik === nedefiniran) odmik = 0;
      var normalL = b2Math.MulTMV(xf.R, normal);
      var offsetL = offset - b2Math.Dot(normal, xf.position);
      var globine = novo Vector_a2j_Number();
      var diveCount = 0;
      var intoIndex = parseInt(-1);
      var outoIndex = parseInt(-1);
      var lastSubmerged = false;
      var i = 0;
      for (i = 0; i < this.m_vertexCount; ++i) {
        globine[i] = b2Matematična.Pika(normalL, this.m_ogl.[i]) - odmikL;
        var isSubmerged = globine[i] < -Number.MIN_VALUE;
        če (i > 0) {
          if (isSubmerged) {
            if (!lastSubmerged) {
              intoIndex = i - 1;
              diveCount++;
            }
          } drugače {
            if (lastSubmerged) {
              outoIndex = i - 1;
              diveCount++;
            }
          }
        }
        lastSubmerged = isSubmerged;
      }
      stikalo (diveCount) {
        primer 0:
          if (lastSubmerged) {
            var md = new b2MassData();
            this.ComputeMass(md, 1);
            c.SetV(b2Math.MulX(xf, md.center));
            vrnitev md.mass;
          } drugače {
            vrni 0;
          }
          odmor;
        primer 1:
          if (intoIndex == -1) {
            intoIndex = this.m_vertexCount - 1;
          } drugače {
            outoIndex = this.m_vertexCount - 1;
          }
          odmor;
      }
      var intoIndex2 = parseInt((intoIndex + 1) % this.m_vertexCount);
      var outoIndex2 = parseInt((outoIndex + 1) % this.m_vertexCount);
      var intoLamdda =
        (0 - globine[intoIndex]) / (globine[intoIndex2] - globine[intoIndex]);
      var outoLamdda =
        (0 - globine[outoIndex]) / (globine[outoIndex2] - globine[outoIndex]);
      var intoVec = novo b2Vec2(
        this.m_vertices[intoIndex].x * (1 - intoLamdda) +
          this.m_vertices[intoIndex2].x * intoLamdda,
        this.m_vertices[intoIndex].y * (1 - intoLamdda) +
          this.m_vertices[intoIndex2].y * intoLamdda
      );
      var outoVec = novo b2Vec2(
        this.m_vertices[outoIndex].x * (1 - outoLamdda) +
          this.m_vertices[outoIndex2].x * outoLamdda,
        this.m_vertices[outoIndex].y * (1 - outoLamdda) +
          this.m_vertices[outoIndex2].y * outoLamdda
      );
      spremenljivo območje = 0;
      var center = novo b2Vec2();
      var p2 = this.m_vertices[intoIndex2];
      var p3;
      i = v Indeks2;
      medtem ko (i != outoIndex2) {
        i = (i + 1) % this.m_vertexCount;
        if (i == outoIndex2) p3 = outoVec;
        else p3 = this.m_vertices[i];
        var triangleArea =
          0,5 *
          ((p2.x - intoVec.x) * (p3.y - intoVec.y) -
            (p2.y - intoVec.y) * (p3.x - intoVec.x));
        površina += trikotnikArea;
        center.x += (trikotnikArea * (intoVec.x + p2.x + p3.x)) / 3;
        center.y += (trikotnikArea * (intoVec.y + p2.y + p3.y)) / 3;
        p2 = p3;
      }
      center.Množi (1 / območje);
      c.SetV(b2Math.MulX(xf, center));
      povratno območje;
    };
    b2PolygonShape.prototype.GetVertexCount = funkcija () {
      vrni this.m_vertexCount;
    };
    b2PolygonShape.prototype.GetVertices = funkcija () {
      vrni this.m_vertices;
    };
    b2PolygonShape.prototype.GetNormals = funkcija () {
      vrni this.m_normals;
    };
    b2PolygonShape.prototype.GetSupport = funkcija (d) {
      var bestIndex = 0;
      var bestValue = this.m_vertices[0].x * dx + this.m_vertices[0].y * dy;
      for (var i = 1; i < this.m_vertexCount; ++i) {
        var value = this.m_vertices[i].x * dx + this.m_vertices[i].y * dy;
        if (value > bestValue) {
          bestIndex = i;
          bestValue = vrednost;
        }
      }
      vrni bestIndex;
    };
    b2PolygonShape.prototype.GetSupportVertex = funkcija (d) {
      var bestIndex = 0;
      var bestValue = this.m_vertices[0].x * dx + this.m_vertices[0].y * dy;
      for (var i = 1; i < this.m_vertexCount; ++i) {
        var value = this.m_vertices[i].x * dx + this.m_vertices[i].y * dy;
        if (value > bestValue) {
          bestIndex = i;
          bestValue = vrednost;
        }
      }
      vrni this.m_vertices[bestIndex];
    };
    b2PolygonShape.prototype.Validate = funkcija () {
      vrni false;
    };
    b2PolygonShape.prototype.b2PolygonShape = funkcija () {
      this.__super.b2Shape.call(this);
      this.m_type = b2Shape.e_polygonShape;
      this.m_centroid = novo b2Vec2();
      this.m_vertices = nov vektor();
      this.m_normals = nov vektor();
    };
    b2PolygonShape.prototype.Reserve = funkcija (štetje) {
      če (štetje === nedefinirano) število = 0;
      for (var i = parseInt(this.m_vertices.length); i < count; i++) {
        this.m_vertices[i] = novo b2Vec2();
        this.m_normals[i] = novo b2Vec2();
      }
    };
    b2PolygonShape.ComputeCentroid = funkcija (vs, štetje) {
      če (štetje === nedefinirano) število = 0;
      var c = novo b2Vec2();
      var površina = 0,0;
      var p1X = 0,0;
      var p1Y = 0,0;
      var inv3 = 1,0 / 3,0;
      for (var i = 0; i < count; ++i) {
        var p2 = vs[i];
        var p3 = i + 1 < štetje? vs[parseInt(i + 1)] : vs[0];
        var e1X = p2.x - p1X;
        var e1Y = p2.y - p1Y;
        var e2X = p3.x - p1X;
        var e2Y = p3.y - p1Y;
        var. D = e1X * e2Y - e1Y * e2X;
        var triangleArea = 0,5 * D;
        površina += trikotnikArea;
        cx += triangleArea * inv3 * (p1X + p2.x + p3.x);
        cy += triangleArea * inv3 * (p1Y + p2.y + p3.y);
      }
      cx *= 1,0 / površina;
      cy *= 1,0 / površina;
      vrnitev c;
    };
    b2PolygonShape.ComputeOBB = funkcija (obb, proti, štetje) {
      če (štetje === nedefinirano) število = 0;
      var i = 0;
      var p = nov vektor (štetje + 1);
      for (i = 0; i < count; ++i) {
        p[i] = vs[i];
      }
      p[štetje] = p[0];
      var minArea = Number.MAX_VALUE;
      za (i = 1; i <= štetje; ++i) {
        var root = p[parseInt(i - 1)];
        var uxX = p[i].x - root.x;
        var uxY = p[i].y - koren.y;
        var length = Math.sqrt(uxX * uxX + uxY * uxY);
        uxX /= dolžina;
        uxY /= dolžina;
        var uyX = -uxY;
        var uyY = uxX;
        var lowerX = Number.MAX_VALUE;
        var lowerY = Number.MAX_VALUE;
        var upperX = -Number.MAX_VALUE;
        var upperY = -Number.MAX_VALUE;
        for (var j = 0; j < count; ++j) {
          var dX = p[j].x - koren.x;
          var dY = p[j].y - koren.y;
          var rX = uxX * dX + uxY * dY;
          var rY = uyX * dX + uyY * dY;
          če (rX < nižji X) nižji X = rX;
          if (rY < nižji Y) nižji Y = rY;
          if (rX > zgornjiX) zgornjiX = rX;
          if (rY > upperY) upperY = rY;
        }
        spremenljivo območje = (zgornji X - spodnji X) * (zgornji Y - spodnji Y);
        if (površina < 0,95 * minArea) {
          minArea = površina;
          obb.R.col1.x = uxX;
          obb.R.col1.y = uxY;
          obb.R.col2.x = uyX;
          obb.R.col2.y = uyY;
          var centerX = 0,5 * (spodnji X + zgornji X);
          var centerY = 0,5 * (spodnji Y + zgornji Y);
          var tMat = obb.R;
          obb.center.x = root.x + (tMat.col1.x * centerX + tMat.col2.x * centerY);
          obb.center.y = root.y + (tMat.col1.y * centerX + tMat.col2.y * centerY);
          obb.extents.x = 0,5 * (zgornji X - spodnji X);
          obb.extents.y = 0,5 * (zgornji Y - spodnji Y);
        }
      }
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Collision.Shapes.b2PolygonShape.s_mat = novo b2Mat22();
    });
    b2Shape.b2Shape = funkcija () {};
    b2Shape.prototype.Copy = funkcija () {
      vrni nič;
    };
    b2Shape.prototype.Set = funkcija (drugo) {
      this.m_radius = other.m_radius;
    };
    b2Shape.prototype.GetType = funkcija () {
      vrni this.m_type;
    };
    b2Shape.prototype.TestPoint = funkcija (xf, p) {
      vrni false;
    };
    b2Shape.prototype.RayCast = funkcija (izhod, vnos, transformacija) {
      vrni false;
    };
    b2Shape.prototype.ComputeAABB = funkcija (aabb, xf) {};
    b2Shape.prototype.ComputeMass = funkcija (massData, density) {
      if (gostota === nedefinirano) gostota = 0;
    };
    b2Shape.prototype.ComputeSubmergedArea = funkcija (normalno, odmik, xf, c) {
      če (odmik === nedefiniran) odmik = 0;
      vrni 0;
    };
    b2Shape.TestOverlap = funkcija (oblika1, transformacija1, oblika2, transformacija2) {
      var input = new b2DistanceInput();
      input.proxyA = novo b2DistanceProxy();
      input.proxyA.Set(shape1);
      input.proxyB = novo b2DistanceProxy();
      input.proxyB.Set(shape2);
      input.transformA = transform1;
      input.transformB = transform2;
      input.useRadii = res;
      var simplexCache = new b2SimplexCache();
      simplexCache.count = 0;
      var output = new b2DistanceOutput();
      b2Distance.Distance(output, simplexCache, input);
      vrni izhod.razdalja < 10,0 * Število.MIN_VREDNOST;
    };
    b2Shape.prototype.b2Shape = funkcija () {
      this.m_type = b2Shape.e_unknownShape;
      this.m_radius = b2Settings.b2_linearSlop;
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Collision.Shapes.b2Shape.e_unknownShape = parseInt(-1);
      Box2D.Collision.Shapes.b2Shape.e_circleShape = 0;
      Box2D.Collision.Shapes.b2Shape.e_polygonShape = 1;
      Box2D.Collision.Shapes.b2Shape.e_edgeShape = 2;
      Box2D.Collision.Shapes.b2Shape.e_shapeTypeCount = 3;
      Box2D.Collision.Shapes.b2Shape.e_hitCollide = 1;
      Box2D.Collision.Shapes.b2Shape.e_missCollide = 0;
      Box2D.Collision.Shapes.b2Shape.e_startsInsideCollide = parseInt(-1);
    });
  })();
  (funkcija () {
    var b2Color = Box2D.Common.b2Color,
      b2notranji = Box2D.Common.b2notranji,
      b2Settings = Box2D.Common.b2Settings,
      b2Mat22 = Box2D.Common.Math.b2Mat22,
      b2Mat33 = Box2D.Common.Math.b2Mat33,
      b2Math = Box2D.Common.Math.b2Math,
      b2Sweep = Box2D.Common.Math.b2Sweep,
      b2Transform = Box2D.Common.Math.b2Transform,
      b2Vec2 = Box2D.Common.Math.b2Vec2,
      b2Vec3 = Box2D.Common.Math.b2Vec3;

    b2Color.b2Color = funkcija () {
      to._r = 0;
      this._g = 0;
      to._b = 0;
    };
    b2Color.prototype.b2Color = funkcija (rr, gg, bb) {
      if (rr === nedefinirano) rr = 0;
      if (gg === nedefinirano) gg = 0;
      if (bb === nedefinirano) bb = 0;
      this._r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0));
      this._g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0));
      this._b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0));
    };
    b2Color.prototype.Set = funkcija (rr, gg, bb) {
      if (rr === nedefinirano) rr = 0;
      if (gg === nedefinirano) gg = 0;
      if (bb === nedefinirano) bb = 0;
      this._r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0));
      this._g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0));
      this._b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0));
    };
    Object.defineProperty(b2Color.prototype, "r", {
      enumerable: false,
      nastavljivo: res,
      set: funkcija (rr) {
        if (rr === nedefinirano) rr = 0;
        this._r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0));
      },
    });
    Object.defineProperty(b2Color.prototype, "g", {
      enumerable: false,
      nastavljivo: res,
      set: funkcija (gg) {
        if (gg === nedefinirano) gg = 0;
        this._g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0));
      },
    });
    Object.defineProperty(b2Color.prototype, "b", {
      enumerable: false,
      nastavljivo: res,
      set: funkcija (bb) {
        if (bb === nedefinirano) bb = 0;
        this._b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0));
      },
    });
    Object.defineProperty(b2Color.prototype, "color", {
      enumerable: false,
      nastavljivo: res,
      get: funkcija () {
        vrnitev (ta._r << 16) | (ta._g << 8) | to._b;
      },
    });
    b2Settings.b2Settings = funkcija () {};
    b2Settings.b2MixFriction = funkcija (trenje1, trenje2) {
      if (trenje1 === nedefinirano) trenje1 = 0;
      if (trenje2 === nedefinirano) trenje2 = 0;
      return Math.sqrt(trenje1 * trenje2);
    };
    b2Settings.b2MixRestitution = funkcija (restitucija1, restitucija2) {
      če (restitucija1 === nedefinirano) restitucija1 = 0;
      če (restitucija2 === nedefinirano) restitucija2 = 0;
      vrniti restitucija1 > restitucija2? restitucija1 : restitucija2;
    };
    b2Settings.b2Assert = funkcija (a) {
      če) {
        vrzi "Trditev ni uspela";
      }
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Common.b2Settings.VERSION = "2.1alpha";
      Box2D.Common.b2Settings.USHRT_MAX = 0x0000ffff;
      Box2D.Common.b2Settings.b2_pi = Math.PI;
      Box2D.Common.b2Settings.b2_maxManifoldPoints = 2;
      Box2D.Common.b2Settings.b2_aabbExtension = 0,1;
      Box2D.Common.b2Settings.b2_aabbMultiplier = 2,0;
      Box2D.Common.b2Settings.b2_polygonRadius = 2,0 * b2Settings.b2_linearSlop;
      Box2D.Common.b2Settings.b2_linearSlop = 0,005;
      Box2D.Common.b2Settings.b2_angularSlop = (2,0 / 180,0) * b2Settings.b2_pi;
      Box2D.Common.b2Settings.b2_toiSlop = 8,0 * b2Settings.b2_linearSlop;
      Box2D.Common.b2Settings.b2_maxTOIContactsPerIsland = 32;
      Box2D.Common.b2Settings.b2_maxTOIJointsPerIsland = 32;
      Box2D.Common.b2Settings.b2_velocityThreshold = 1,0;
      Box2D.Common.b2Settings.b2_maxLinearCorrection = 0,2;
      Box2D.Common.b2Settings.b2_maxAngularCorrection =
        (8,0 / 180,0) * b2Settings.b2_pi;
      Box2D.Common.b2Settings.b2_maxTranslation = 2.0;
      Box2D.Common.b2Settings.b2_maxTranslationSquared =
        b2Settings.b2_maxTranslation * b2Settings.b2_maxTranslation;
      Box2D.Common.b2Settings.b2_maxRotation = 0,5 * b2Settings.b2_pi;
      Box2D.Common.b2Settings.b2_maxRotationSquared =
        b2Settings.b2_maxRotation * b2Settings.b2_maxRotation;
      Box2D.Common.b2Settings.b2_contactBaumgarte = 0,2;
      Box2D.Common.b2Settings.b2_timeToSleep = 0,5;
      Box2D.Common.b2Settings.b2_linearSleepTolerance = 0,01;
      Box2D.Common.b2Settings.b2_angularSleepTolerance =
        (2,0 / 180,0) * b2Settings.b2_pi;
    });
  })();
  (funkcija () {
    var b2AABB = Box2D.Collision.b2AABB,
      b2Color = Box2D.Common.b2Color,
      b2notranji = Box2D.Common.b2notranji,
      b2Settings = Box2D.Common.b2Settings,
      b2Mat22 = Box2D.Common.Math.b2Mat22,
      b2Mat33 = Box2D.Common.Math.b2Mat33,
      b2Math = Box2D.Common.Math.b2Math,
      b2Sweep = Box2D.Common.Math.b2Sweep,
      b2Transform = Box2D.Common.Math.b2Transform,
      b2Vec2 = Box2D.Common.Math.b2Vec2,
      b2Vec3 = Box2D.Common.Math.b2Vec3;

    b2Mat22.b2Mat22 = funkcija () {
      this.col1 = novo b2Vec2();
      this.col2 = novo b2Vec2();
    };
    b2Mat22.prototype.b2Mat22 = funkcija () {
      this.SetIdentity();
    };
    b2Mat22.FromAngle = funkcija (kot) {
      if (kot === nedefiniran) kot = 0;
      var mat = novo b2Mat22();
      mat.Set(kot);
      povratna podloga;
    };
    b2Mat22.FromVV = funkcija (c1, c2) {
      var mat = novo b2Mat22();
      mat.SetVV(c1, c2);
      povratna podloga;
    };
    b2Mat22.prototype.Set = funkcija (kot) {
      if (kot === nedefiniran) kot = 0;
      var c = Math.cos(kot);
      var s = Math.sin(kot);
      this.col1.x = c;
      this.col2.x = -s;
      this.col1.y = s;
      this.col2.y = c;
    };
    b2Mat22.prototype.SetVV = funkcija (c1, c2) {
      this.col1.SetV(c1);
      this.col2.SetV(c2);
    };
    b2Mat22.prototype.Copy = funkcija () {
      var mat = novo b2Mat22();
      mat.SetM(to);
      povratna podloga;
    };
    b2Mat22.prototype.SetM = funkcija (m) {
      this.col1.SetV(m.col1);
      this.col2.SetV(m.col2);
    };
    b2Mat22.prototype.AddM = funkcija (m) {
      this.col1.x += m.col1.x;
      this.col1.y += m.col1.y;
      this.col2.x += m.col2.x;
      this.col2.y += m.col2.y;
    };
    b2Mat22.prototype.SetIdentity = funkcija () {
      this.col1.x = 1,0;
      this.col2.x = 0,0;
      this.col1.y = 0,0;
      this.col2.y = 1,0;
    };
    b2Mat22.prototype.SetZero = funkcija () {
      this.col1.x = 0,0;
      this.col2.x = 0,0;
      this.col1.y = 0,0;
      this.col2.y = 0,0;
    };
    b2Mat22.prototype.GetAngle = funkcija () {
      vrni Math.atan2(this.col1.y, this.col1.x);
    };
    b2Mat22.prototype.GetInverse = funkcija (out) {
      var a = this.col1.x;
      var b = this.col2.x;
      var c = this.col1.y;
      var d = this.col2.y;
      var det = a * d - b * c;
      if (det != 0.0) {
        det = 1,0 / det;
      }
      out.col1.x = det * d;
      out.col2.x = -det * b;
      out.col1.y = -det * c;
      out.col2.y = det * a;
      vrnitev ven;
    };
    b2Mat22.prototype.Solve = funkcija (out, bX, bY) {
      if (bX === nedefinirano) bX = 0;
      if (bY === nedefinirano) bY = 0;
      var a11 = this.col1.x;
      var a12 = this.col2.x;
      var a21 = this.col1.y;
      var a22 = this.col2.y;
      var det = a11 * a22 - a12 * a21;
      if (det != 0.0) {
        det = 1,0 / det;
      }
      out.x = det * (a22 * bX - a12 * bY);
      out.y = det * (a11 * bY - a21 * bX);
      vrnitev ven;
    };
    b2Mat22.prototype.Abs = funkcija () {
      this.col1.Abs();
      this.col2.Abs();
    };
    b2Mat33.b2Mat33 = funkcija () {
      this.col1 = novo b2Vec3();
      this.col2 = novo b2Vec3();
      this.col3 = novo b2Vec3();
    };
    b2Mat33.prototype.b2Mat33 = funkcija (c1, c2, c3) {
      če (c1 === nedefinirano) c1 = nič;
      če (c2 === nedefinirano) c2 = nič;
      če (c3 === nedefinirano) c3 = nič;
      if (!c1 && !c2 && !c3) {
        this.col1.SetZero();
        this.col2.SetZero();
        this.col3.SetZero();
      } drugače {
        this.col1.SetV(c1);
        this.col2.SetV(c2);
        this.col3.SetV(c3);
      }
    };
    b2Mat33.prototype.SetVVV = funkcija (c1, c2, c3) {
      this.col1.SetV(c1);
      this.col2.SetV(c2);
      this.col3.SetV(c3);
    };
    b2Mat33.prototype.Copy = funkcija () {
      vrni novo b2Mat33(this.col1, this.col2, this.col3);
    };
    b2Mat33.prototype.SetM = funkcija (m) {
      this.col1.SetV(m.col1);
      this.col2.SetV(m.col2);
      this.col3.SetV(m.col3);
    };
    b2Mat33.prototype.AddM = funkcija (m) {
      this.col1.x += m.col1.x;
      this.col1.y += m.col1.y;
      this.col1.z += m.col1.z;
      this.col2.x += m.col2.x;
      this.col2.y += m.col2.y;
      this.col2.z += m.col2.z;
      this.col3.x += m.col3.x;
      this.col3.y += m.col3.y;
      this.col3.z += m.col3.z;
    };
    b2Mat33.prototype.SetIdentity = funkcija () {
      this.col1.x = 1,0;
      this.col2.x = 0,0;
      this.col3.x = 0,0;
      this.col1.y = 0,0;
      this.col2.y = 1,0;
      this.col3.y = 0,0;
      this.col1.z = 0,0;
      this.col2.z = 0,0;
      this.col3.z = 1,0;
    };
    b2Mat33.prototype.SetZero = funkcija () {
      this.col1.x = 0,0;
      this.col2.x = 0,0;
      this.col3.x = 0,0;
      this.col1.y = 0,0;
      this.col2.y = 0,0;
      this.col3.y = 0,0;
      this.col1.z = 0,0;
      this.col2.z = 0,0;
      this.col3.z = 0,0;
    };
    b2Mat33.prototype.Solve22 = funkcija (out, bX, bY) {
      if (bX === nedefinirano) bX = 0;
      if (bY === nedefinirano) bY = 0;
      var a11 = this.col1.x;
      var a12 = this.col2.x;
      var a21 = this.col1.y;
      var a22 = this.col2.y;
      var det = a11 * a22 - a12 * a21;
      if (det != 0.0) {
        det = 1,0 / det;
      }
      out.x = det * (a22 * bX - a12 * bY);
      out.y = det * (a11 * bY - a21 * bX);
      vrnitev ven;
    };
    b2Mat33.prototype.Solve33 = funkcija (out, bX, bY, bZ) {
      if (bX === nedefinirano) bX = 0;
      if (bY === nedefinirano) bY = 0;
      if (bZ === nedefinirano) bZ = 0;
      var a11 = this.col1.x;
      var a21 = this.col1.y;
      var a31 = this.col1.z;
      var a12 = this.col2.x;
      var a22 = this.col2.y;
      var a32 = this.col2.z;
      var a13 = this.col3.x;
      var a23 = this.col3.y;
      var a33 = this.col3.z;
      var det =
        a11 * (a22 * a33 - a32 * a23) +
        a21 * (a32 * a13 - a12 * a33) +
        a31 * (a12 * a23 - a22 * a13);
      if (det != 0.0) {
        det = 1,0 / det;
      }
      ven.x =
        det *
        (bX * (a22 * a33 - a32 * a23) +
          bY * (a32 * a13 - a12 * a33) +
          bZ * (a12 * a23 - a22 * a13));
      ven.y =
        det *
        (a11 * (bY * a33 - bZ * a23) +
          a21 * (bZ * a13 - bX * a33) +
          a31 * (bX * a23 - bY * a13));
      ven.z =
        det *
        (a11 * (a22 * bZ - a32 * bY) +
          a21 * (a32 * bX - a12 * bZ) +
          a31 * (a12 * bY - a22 * bX));
      vrnitev ven;
    };
    b2Math.b2Math = funkcija () {};
    b2Math.IsValid = funkcija (x) {
      if (x === nedefinirano) x = 0;
      return isFinite(x);
    };
    b2Math.Dot = funkcija (a, b) {
      vrni ax * bx + ay * by;
    };
    b2Math.CrossVV = funkcija (a, b) {
      vrni sekiro * po - ay * bx;
    };
    b2Math.CrossVF = funkcija (a, s) {
      if (s === nedefinirano) s = 0;
      var v = novo b2Vec2(s * ay, -s * ax);
      vrnitev v;
    };
    b2Math.CrossFV = funkcija (s, a) {
      if (s === nedefinirano) s = 0;
      var v = novo b2Vec2(-s * ay, s * ax);
      vrnitev v;
    };
    b2Math.MulMV = funkcija (A, v) {
      var u = novo b2Vec2(
        A.col1.x * vx + A.col2.x * vy,
        A.col1.y * vx + A.col2.y * vy
      );
      vrni u;
    };
    b2Math.MulTMV = funkcija (A, v) {
      var u = novo b2Vec2(b2Math.Dot(v, A.col1), b2Math.Dot(v, A.col2));
      vrni u;
    };
    b2Math.MulX = funkcija (T, v) {
      var a = b2Math.MulMV(TR, v);
      ax += T.position.x;
      ay += T.position.y;
      vrnitev a;
    };
    b2Math.MulXT = funkcija (T, v) {
      var a = b2Math.SubtractVV(v, T.position);
      var tX = ax * TRcol1.x + ay * TRcol1.y;
      ay = ax * TRcol2.x + ay * TRcol2.y;
      ax = tX;
      vrnitev a;
    };
    b2Math.AddVV = funkcija (a, b) {
      var v = novo b2Vec2(ax + bx, ay + by);
      vrnitev v;
    };
    b2Math.SubtractVV = funkcija (a, b) {
      var v = novo b2Vec2(ax - bx, ay - by);
      vrnitev v;
    };
    b2Math.Distance = funkcija (a, b) {
      var cX = ax - bx;
      var cY = ay - by;
      return Math.sqrt(cX * cX + cY * cY);
    };
    b2Math.DistanceSquared = funkcija (a, b) {
      var cX = ax - bx;
      var cY = ay - by;
      vrnitev cX * cX + cY * cY;
    };
    b2Math.MulFV = funkcija (s, a) {
      if (s === nedefinirano) s = 0;
      var v = novo b2Vec2(s * ax, s * ay);
      vrnitev v;
    };
    b2Math.AddMM = funkcija (A, B) {
      var C = b2Mat22.FromVV(
        b2Math.AddVV(A.col1, B.col1),
        b2Math.AddVV(A.col2, B.col2)
      );
      vrnitev C;
    };
    b2Math.MulMM = funkcija (A, B) {
      var C = b2Mat22.FromVV(b2Math.MulMV(A, B.col1), b2Math.MulMV(A, B.col2));
      vrnitev C;
    };
    b2Math.MulTMM = funkcija (A, B) {
      var c1 = novo b2Vec2(b2Math.Dot(A.col1, B.col1), b2Math.Dot(A.col2, B.col1));
      var c2 = novo b2Vec2(b2Math.Dot(A.col1, B.col2), b2Math.Dot(A.col2, B.col2));
      var C = b2Mat22.FromVV(c1, c2);
      vrnitev C;
    };
    b2Math.Abs ​​= funkcija (a) {
      if (a === nedefinirano) a = 0;
      vrni a > 0,0? a : -a;
    };
    b2Math.AbsV = funkcija (a) {
      var b = novo b2Vec2(b2Math.Abs(ax), b2Math.Abs(ay));
      vrnitev b;
    };
    b2Math.AbsM = funkcija (A) {
      var B = b2Mat22.FromVV(b2Math.AbsV(A.col1), b2Math.AbsV(A.col2));
      vrnitev B;
    };
    b2Math.Min = funkcija (a, b) {
      if (a === nedefinirano) a = 0;
      if (b === nedefinirano) b = 0;
      vrni a < b? a : b;
    };
    b2Math.MinV = funkcija (a, b) {
      var c = novo b2Vec2(b2Math.Min(ax, bx), b2Math.Min(ay, by));
      vrnitev c;
    };
    b2Math.Max ​​= funkcija (a, b) {
      if (a === nedefinirano) a = 0;
      if (b === nedefinirano) b = 0;
      vrni a > b? a : b;
    };
    b2Math.MaxV = funkcija (a, b) {
      var c = novo b2Vec2(b2Math.Max(ax, bx), b2Math.Max(ay, by));
      vrnitev c;
    };
    b2Math.Clamp = funkcija (a, nizka, visoka) {
      if (a === nedefinirano) a = 0;
      če (nizko === nedefinirano) nizko = 0;
      če (visoko === nedefinirano) visoko = 0;
      vrni <nizko? nizko : a > visoko ? visoko: a;
    };
    b2Math.ClampV = funkcija (a, nizka, visoka) {
      return b2Math.MaxV(low, b2Math.MinV(a, high));
    };
    b2Math.Swap = funkcija (a, b) {
      var tmp = a[0];
      a[0] = b[0];
      b[0] = tmp;
    };
    b2Math.Random = funkcija () {
      return Math.random() * 2 - 1;
    };
    b2Math.RandomRange = funkcija (lo, hi) {
      if (lo === nedefinirano) lo = 0;
      if (hi === nedefinirano) hi = 0;
      var r = Math.random();
      r = (hi - lo) * r + lo;
      vrnitev r;
    };
    b2Math.NextPowerOfTwo = funkcija (x) {
      if (x === nedefinirano) x = 0;
      x |= (x >> 1) & 0x7fffffff;
      x |= (x >> 2) & 0x3fffffff;
      x |= (x >> 4) & 0x0fffffff;
      x |= (x >> 8) & 0x00ffffff;
      x |= (x >> 16) & 0x0000ffff;
      vrni x + 1;
    };
    b2Math.IsPowerOfTwo = funkcija (x) {
      if (x === nedefinirano) x = 0;
      var rezultat = x > 0 && (x & (x - 1)) == 0;
      vrni rezultat;
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Common.Math.b2Math.b2Vec2_zero = novo b2Vec2(0,0, 0,0);
      Box2D.Common.Math.b2Math.b2Mat22_identity = b2Mat22.FromVV(
        novo b2Vec2(1.0, 0.0),
        novo b2Vec2(0.0, 1.0)
      );
      Box2D.Common.Math.b2Math.b2Transform_identity = novo b2Transform(
        b2Math.b2Vec2_zero,
        b2Math.b2Mat22_identiteta
      );
    });
    b2Sweep.b2Sweep = funkcija () {
      this.localCenter = novo b2Vec2();
      this.c0 = novo b2Vec2();
      this.c = novo b2Vec2();
    };
    b2Sweep.prototype.Set = funkcija (drugo) {
      this.localCenter.SetV(other.localCenter);
      this.c0.SetV(other.c0);
      this.c.SetV(other.c);
      this.a0 = other.a0;
      to.a = drugo.a;
      this.t0 = other.t0;
    };
    b2Sweep.prototype.Copy = funkcija () {
      var copy = new b2Sweep();
      copy.localCenter.SetV(this.localCenter);
      copy.c0.SetV(this.c0);
      copy.c.SetV(this.c);
      kopija.a0 = to.a0;
      kopija.a = to.a;
      copy.t0 = this.t0;
      povratni izvod;
    };
    b2Sweep.prototype.GetTransform = funkcija (xf, alfa) {
      če (alfa === nedefinirano) alfa = 0;
      xf.position.x = (1.0 - alfa) * this.c0.x + alfa * this.cx;
      xf.position.y = (1,0 - alfa) * this.c0.y + alfa * this.cy;
      spremenljiv kot = (1,0 - alfa) * this.a0 + alpha * this.a;
      xf.R.Set(kot);
      var tMat = xf.R;
      xf.položaj.x -=
        tMat.col1.x * this.localCenter.x + tMat.col2.x * this.localCenter.y;
      xf.position.y -=
        tMat.col1.y * this.localCenter.x + tMat.col2.y * this.localCenter.y;
    };
    b2Sweep.prototype.Advance = funkcija (t) {
      if (t === nedefinirano) t = 0;
      if (this.t0 < t && 1.0 - this.t0 > Number.MIN_VALUE) {
        var alpha = (t - this.t0) / (1.0 - this.t0);
        this.c0.x = (1.0 - alfa) * this.c0.x + alfa * this.cx;
        this.c0.y = (1.0 - alfa) * this.c0.y + alfa * this.cy;
        this.a0 = (1,0 - alfa) * this.a0 + alpha * this.a;
        this.t0 = t;
      }
    };
    b2Transform.b2Transform = funkcija () {
      this.position = new b2Vec2();
      this.R = novo b2Mat22();
    };
    b2Transform.prototype.b2Transform = funkcija (pos, r) {
      if (pos === nedefinirano) pos = null;
      če (r === nedefinirano) r = nič;
      if (pos) {
        this.position.SetV(pos);
        this.R.SetM(r);
      }
    };
    b2Transform.prototype.Initialize = funkcija (pos, r) {
      this.position.SetV(pos);
      this.R.SetM(r);
    };
    b2Transform.prototype.SetIdentity = funkcija () {
      this.position.SetZero();
      this.R.SetIdentity();
    };
    b2Transform.prototype.Set = funkcija (x) {
      this.position.SetV(x.position);
      to.R.SetM(xR);
    };
    b2Transform.prototype.GetAngle = funkcija () {
      vrni Math.atan2(this.R.col1.y, this.R.col1.x);
    };
    b2Vec2.b2Vec2 = funkcija () {};
    b2Vec2.prototype.b2Vec2 = funkcija (x_, y_) {
      if (x_ === nedefinirano) x_ = 0;
      if (y_ === nedefinirano) y_ = 0;
      to.x = x_;
      this.y = y_;
    };
    b2Vec2.prototype.SetZero = funkcija () {
      this.x = 0,0;
      this.y = 0,0;
    };
    b2Vec2.prototype.Set = funkcija (x_, y_) {
      if (x_ === nedefinirano) x_ = 0;
      if (y_ === nedefinirano) y_ = 0;
      to.x = x_;
      this.y = y_;
    };
    b2Vec2.prototype.SetV = funkcija (v) {
      this.x = vx;
      this.y = vy;
    };
    b2Vec2.prototype.GetNegative = funkcija () {
      vrni novo b2Vec2(-this.x, -this.y);
    };
    b2Vec2.prototype.NegativeSelf = funkcija () {
      ta.x = -ta.x;
      ta.y = -ta.y;
    };
    b2Vec2.Make = funkcija (x_, y_) {
      if (x_ === nedefinirano) x_ = 0;
      if (y_ === nedefinirano) y_ = 0;
      vrni novo b2Vec2(x_, y_);
    };
    b2Vec2.prototype.Copy = funkcija () {
      vrni novo b2Vec2(this.x, this.y);
    };
    b2Vec2.prototype.Add = funkcija (v) {
      this.x += vx;
      this.y += vy;
    };
    b2Vec2.prototype.Subtract = funkcija (v) {
      this.x -= vx;
      this.y -= vy;
    };
    b2Vec2.prototype.Multiply = funkcija (a) {
      if (a === nedefinirano) a = 0;
      this.x *= a;
      this.y *= a;
    };
    b2Vec2.prototype.MulM = funkcija (A) {
      var tX = this.x;
      this.x = A.col1.x * tX + A.col2.x * this.y;
      this.y = A.col1.y * tX + A.col2.y * this.y;
    };
    b2Vec2.prototype.MulTM = funkcija (A) {
      var tX = b2Math.Dot(this, A.col1);
      this.y = b2Math.Dot(this, A.col2);
      this.x = tX;
    };
    b2Vec2.prototype.CrossVF = funkcija(e) {
      if (s === nedefinirano) s = 0;
      var tX = this.x;
      this.x = s * this.y;
      this.y = -s * tX;
    };
    b2Vec2.prototype.CrossFV = funkcija(e) {
      if (s === nedefinirano) s = 0;
      var tX = this.x;
      this.x = -s * this.y;
      this.y = s * tX;
    };
    b2Vec2.prototype.MinV = funkcija (b) {
      to.x = to.x < bx? to.x : bx;
      this.y = this.y < by ? this.y : by;
    };
    b2Vec2.prototype.MaxV = funkcija (b) {
      to.x = to.x > bx? to.x : bx;
      this.y = this.y > by ? this.y : by;
    };
    b2Vec2.prototype.Abs = funkcija () {
      if (this.x < 0) this.x = -this.x;
      if (this.y < 0) this.y = -this.y;
    };
    b2Vec2.prototype.Length = funkcija () {
      return Math.sqrt(this.x * this.x + this.y * this.y);
    };
    b2Vec2.prototype.LengthSquared = funkcija () {
      vrni to.x * to.x + to.y * to.y;
    };
    b2Vec2.prototype.Normalize = funkcija () {
      var length = Math.sqrt(this.x * this.x + this.y * this.y);
      if (length < Number.MIN_VALUE) {
        vrnitev 0,0;
      }
      var invLength = 1,0 / dolžina;
      this.x *= invLength;
      this.y *= invLength;
      povratna dolžina;
    };
    b2Vec2.prototype.IsValid = funkcija () {
      return b2Math.IsValid(this.x) && b2Math.IsValid(this.y);
    };
    b2Vec3.b2Vec3 = funkcija () {};
    b2Vec3.prototype.b2Vec3 = funkcija (x, y, z) {
      if (x === nedefinirano) x = 0;
      če (y === nedefinirano) y = 0;
      if (z === nedefinirano) z = 0;
      to.x = x;
      this.y = y;
      to.z = z;
    };
    b2Vec3.prototype.SetZero = funkcija () {
      to.x = to.y = to.z = 0,0;
    };
    b2Vec3.prototype.Set = funkcija (x, y, z) {
      if (x === nedefinirano) x = 0;
      če (y === nedefinirano) y = 0;
      if (z === nedefinirano) z = 0;
      to.x = x;
      this.y = y;
      to.z = z;
    };
    b2Vec3.prototype.SetV = funkcija (v) {
      this.x = vx;
      this.y = vy;
      to.z = vz;
    };
    b2Vec3.prototype.GetNegative = funkcija () {
      vrni novo b2Vec3(-this.x, -this.y, -this.z);
    };
    b2Vec3.prototype.NegativeSelf = funkcija () {
      ta.x = -ta.x;
      ta.y = -ta.y;
      to.z = -ta.z;
    };
    b2Vec3.prototype.Copy = funkcija () {
      vrni novo b2Vec3(this.x, this.y, this.z);
    };
    b2Vec3.prototype.Add = funkcija (v) {
      this.x += vx;
      this.y += vy;
      this.z += vz;
    };
    b2Vec3.prototype.Subtract = funkcija (v) {
      this.x -= vx;
      this.y -= vy;
      this.z -= vz;
    };
    b2Vec3.prototype.Multiply = funkcija (a) {
      if (a === nedefinirano) a = 0;
      this.x *= a;
      this.y *= a;
      this.z *= a;
    };
  })();
  (funkcija () {
    var b2ControllerEdge = Box2D.Dynamics.Controllers.b2ControllerEdge,
      b2Mat22 = Box2D.Common.Math.b2Mat22,
      b2Mat33 = Box2D.Common.Math.b2Mat33,
      b2Math = Box2D.Common.Math.b2Math,
      b2Sweep = Box2D.Common.Math.b2Sweep,
      b2Transform = Box2D.Common.Math.b2Transform,
      b2Vec2 = Box2D.Common.Math.b2Vec2,
      b2Vec3 = Box2D.Common.Math.b2Vec3,
      b2Color = Box2D.Common.b2Color,
      b2notranji = Box2D.Common.b2notranji,
      b2Settings = Box2D.Common.b2Settings,
      b2AABB = Box2D.Collision.b2AABB,
      b2Bound = Box2D.Collision.b2Bound,
      b2BoundValues ​​= Box2D.Collision.b2BoundValues,
      b2Collision = Box2D.Collision.b2Collision,
      b2ContactID = Box2D.Collision.b2ContactID,
      b2ContactPoint = Box2D.Collision.b2ContactPoint,
      b2Distance = Box2D.Collision.b2Distance,
      b2DistanceInput = Box2D.Collision.b2DistanceInput,
      b2DistanceOutput = Box2D.Collision.b2DistanceOutput,
      b2DistanceProxy = Box2D.Collision.b2DistanceProxy,
      b2DynamicTree = Box2D.Collision.b2DynamicTree,
      b2DynamicTreeBroadPhase = Box2D.Collision.b2DynamicTreeBroadPhase,
      b2DynamicTreeNode = Box2D.Collision.b2DynamicTreeNode,
      b2DynamicTreePair = Box2D.Collision.b2DynamicTreePair,
      b2Manifold = Box2D.Collision.b2Manifold,
      b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint,
      b2Point = Box2D.Collision.b2Point,
      b2RayCastInput = Box2D.Collision.b2RayCastInput,
      b2RayCastOutput = Box2D.Collision.b2RayCastOutput,
      b2Segment = Box2D.Collision.b2Segment,
      b2SeparationFunction = Box2D.Collision.b2SeparationFunction,
      b2Simplex = Box2D.Collision.b2Simplex,
      b2SimplexCache = Box2D.Collision.b2SimplexCache,
      b2SimplexVertex = Box2D.Collision.b2SimplexVertex,
      b2TimeOfImpact = Box2D.Collision.b2TimeOfImpact,
      b2TOIInput = Box2D.Collision.b2TOIInput,
      b2WorldManifold = Box2D.Collision.b2WorldManifold,
      ClipVertex = Box2D.Collision.ClipVertex,
      Lastnosti = Box2D.Collision.Features,
      IBroadPhase = Box2D.Collision.IBroadPhase,
      b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
      b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef,
      b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape,
      b2MassData = Box2D.Collision.Shapes.b2MassData,
      b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
      b2Shape = Box2D.Collision.Shapes.b2Shape,
      b2Body = Box2D.Dynamics.b2Body,
      b2BodyDef = Box2D.Dynamics.b2BodyDef,
      b2ContactFilter = Box2D.Dynamics.b2ContactFilter,
      b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse,
      b2ContactListener = Box2D.Dynamics.b2ContactListener,
      b2ContactManager = Box2D.Dynamics.b2ContactManager,
      b2DebugDraw = Box2D.Dynamics.b2DebugDraw,
      b2DestructionListener = Box2D.Dynamics.b2DestructionListener,
      b2FilterData = Box2D.Dynamics.b2FilterData,
      b2Fixture = Box2D.Dynamics.b2Fixture,
      b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
      b2Island = Box2D.Dynamics.b2Island,
      b2TimeStep = Box2D.Dynamics.b2TimeStep,
      b2World = Box2D.Dynamics.b2World,
      b2CircleContact = Box2D.Dynamics.Contacts.b2CircleContact,
      b2Contact = Box2D.Dynamics.Contacts.b2Contact,
      b2ContactConstraint = Box2D.Dynamics.Contacts.b2ContactConstraint,
      b2ContactConstraintPoint = Box2D.Dynamics.Contacts.b2ContactConstraintPoint,
      b2ContactEdge = Box2D.Dynamics.Contacts.b2ContactEdge,
      b2ContactFactory = Box2D.Dynamics.Contacts.b2ContactFactory,
      b2ContactRegister = Box2D.Dynamics.Contacts.b2ContactRegister,
      b2ContactResult = Box2D.Dynamics.Contacts.b2ContactResult,
      b2ContactSolver = Box2D.Dynamics.Contacts.b2ContactSolver,
      b2EdgeAndCircleContact = Box2D.Dynamics.Contacts.b2EdgeAndCircleContact,
      b2NullContact = Box2D.Dynamics.Contacts.b2NullContact,
      b2PolyAndCircleContact = Box2D.Dynamics.Contacts.b2PolyAndCircleContact,
      b2PolyAndEdgeContact = Box2D.Dynamics.Contacts.b2PolyAndEdgeContact,
      b2PolygonContact = Box2D.Dynamics.Contacts.b2PolygonContact,
      b2PositionSolverManifold = Box2D.Dynamics.Contacts.b2PositionSolverManifold,
      b2Controller = Box2D.Dynamics.Controllers.b2Controller,
      b2DistanceJoint = Box2D.Dynamics.Joints.b2DistanceJoint,
      b2DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef,
      b2FrictionJoint = Box2D.Dynamics.Joints.b2FrictionJoint,
      b2FrictionJointDef = Box2D.Dynamics.Joints.b2FrictionJointDef,
      b2GearJoint = Box2D.Dynamics.Joints.b2GearJoint,
      b2GearJointDef = Box2D.Dynamics.Joints.b2GearJointDef,
      b2Jacobian = Box2D.Dynamics.Joints.b2Jacobian,
      b2Joint = Box2D.Dynamics.Joints.b2Joint,
      b2JointDef = Box2D.Dynamics.Joints.b2JointDef,
      b2JointEdge = Box2D.Dynamics.Joints.b2JointEdge,
      b2LineJoint = Box2D.Dynamics.Joints.b2LineJoint,
      b2LineJointDef = Box2D.Dynamics.Joints.b2LineJointDef,
      b2MouseJoint = Box2D.Dynamics.Joints.b2MouseJoint,
      b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef,
      b2PrismaticJoint = Box2D.Dynamics.Joints.b2PrismaticJoint,
      b2PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef,
      b2PulleyJoint = Box2D.Dynamics.Joints.b2PulleyJoint,
      b2PulleyJointDef = Box2D.Dynamics.Joints.b2PulleyJointDef,
      b2RevoluteJoint = Box2D.Dynamics.Joints.b2RevoluteJoint,
      b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef,
      b2WeldJoint = Box2D.Dynamics.Joints.b2WeldJoint,
      b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef;

    b2Body.b2Body = funkcija () {
      this.m_xf = novo b2Transform();
      this.m_sweep = novo b2Sweep();
      this.m_linearVelocity = novo b2Vec2();
      this.m_force = novo b2Vec2();
    };
    b2Body.prototype.connectEdges = funkcija (s1, s2, kot1) {
      če (kot1 === nedefiniran) kot1 = 0;
      var angle2 = Math.atan2(
        s2.GetDirectionVector().y,
        s2.GetDirectionVector().x
      );
      var coreOffset = Math.tan((kot2 - kot1) * 0,5);
      var core = b2Math.MulFV(coreOffset, s2.GetDirectionVector());
      jedro = b2Math.SubtractVV(jedro, s2.GetNormalVector());
      jedro = b2Math.MulFV(b2Settings.b2_toiSlop, jedro);
      jedro = b2Math.AddVV(jedro, s2.GetVertex1());
      var cornerDir = b2Math.AddVV(
        s1.GetDirectionVector(),
        s2.GetDirectionVector()
      );
      kotniDir.Normaliziraj();
      var convex =
        b2Math.Dot(s1.GetDirectionVector(), s2.GetNormalVector()) > 0,0;
      s1.SetNextEdge(s2, jedro, kotniDir, konveksen);
      s2.SetPrevEdge(s1, core, cornerDir, convex);
      povratni kot2;
    };
    b2Body.prototype.CreateFixture = funkcija (def) {
      if (this.m_world.IsLocked() == true) {
        vrni nič;
      }
      var fixture = new b2Fixture();
      fixture.Create(this, this.m_xf, def);
      if (this.m_flags & b2Body.e_activeFlag) {
        var broadPhase = this.m_world.m_contactManager.m_broadPhase;
        fixture.CreateProxy(broadPhase, this.m_xf);
      }
      fixture.m_next = this.m_fixtureList;
      this.m_fixtureList = stalnica;
      ++this.m_fixtureCount;
      fixture.m_body = to;
      if (fixture.m_density > 0,0) {
        this.ResetMassData();
      }
      this.m_world.m_flags |= b2World.e_newFixture;
      povratna napeljava;
    };
    b2Body.prototype.CreateFixture2 = funkcija (oblika, gostota) {
      if (gostota === nedefinirano) gostota = 0,0;
      var def = novo b2FixtureDef();
      def.shape = oblika;
      def.density = gostota;
      vrni to.CreateFixture(def);
    };
    b2Body.prototype.DestroyFixture = funkcija (fiksa) {
      if (this.m_world.IsLocked() == true) {
        vrnitev;
      }
      var node = this.m_fixtureList;
      var ppF = nič;
      najdena var = false;
      medtem ko (vozlišče != nič) {
        če (vozlišče == stalnica) {
          if (ppF) ppF.m_next = fixture.m_next;
          sicer this.m_fixtureList = fixture.m_next;
          najdeno = res;
          odmor;
        }
        ppF = vozlišče;
        vozlišče = vozlišče.m_naslednji;
      }
      var edge = this.m_contactList;
      medtem ko (rob) {
        var c = edge.contact;
        rob = rob.naslednji;
        var fixtureA = c.GetFixtureA();
        var fixtureB = c.GetFixtureB();
        if (fiks == fixtureA || fixture == fixtureB) {
          this.m_world.m_contactManager.Destroy(c);
        }
      }
      if (this.m_flags & b2Body.e_activeFlag) {
        var broadPhase = this.m_world.m_contactManager.m_broadPhase;
        fixture.DestroyProxy(broadPhase);
      } drugače {
      }
      napeljava.Uniči();
      fixture.m_body = null;
      fixture.m_next = nič;
      --this.m_fixtureCount;
      this.ResetMassData();
    };
    b2Body.prototype.SetPositionAndAngle = funkcija (položaj, kot) {
      if (kot === nedefiniran) kot = 0;
      var f;
      if (this.m_world.IsLocked() == true) {
        vrnitev;
      }
      this.m_xf.R.Set(kot);
      this.m_xf.position.SetV(položaj);
      var tMat = this.m_xf.R;
      var tVec = this.m_sweep.localCenter;
      this.m_sweep.cx = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      this.m_sweep.cy = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      this.m_sweep.cx += this.m_xf.position.x;
      this.m_sweep.cy += this.m_xf.position.y;
      this.m_sweep.c0.SetV(this.m_sweep.c);
      this.m_sweep.a0 = this.m_sweep.a = kot;
      var broadPhase = this.m_world.m_contactManager.m_broadPhase;
      za (f = this.m_fixtureList; f; f = f.m_next) {
        f.Synchronize(broadPhase, this.m_xf, this.m_xf);
      }
      this.m_world.m_contactManager.FindNewContacts();
    };
    b2Body.prototype.SetTransform = funkcija (xf) {
      this.SetPositionAndAngle(xf.position, xf.GetAngle());
    };
    b2Body.prototype.GetTransform = funkcija () {
      vrni to.m_xf;
    };
    b2Body.prototype.GetPosition = funkcija () {
      vrni this.m_xf.position;
    };
    b2Body.prototype.SetPosition = funkcija (položaj) {
      this.SetPositionAndAngle(position, this.GetAngle());
    };
    b2Body.prototype.GetAngle = funkcija () {
      vrni this.m_sweep.a;
    };
    b2Body.prototype.SetAngle = funkcija (kota) {
      if (kot === nedefiniran) kot = 0;
      this.SetPositionAndAngle(this.GetPosition(), kot);
    };
    b2Body.prototype.GetWorldCenter = funkcija () {
      vrni this.m_sweep.c;
    };
    b2Body.prototype.GetLocalCenter = funkcija () {
      vrni this.m_sweep.localCenter;
    };
    b2Body.prototype.SetLinearVelocity = funkcija (v) {
      if (this.m_type == b2Body.b2_staticBody) {
        vrnitev;
      }
      this.m_linearVelocity.SetV(v);
    };
    b2Body.prototype.GetLinearVelocity = funkcija () {
      vrni this.m_linearVelocity;
    };
    b2Body.prototype.SetAngularVelocity = funkcija (omega) {
      če (omega === nedefinirano) omega = 0;
      if (this.m_type == b2Body.b2_staticBody) {
        vrnitev;
      }
      this.m_angularVelocity = omega;
    };
    b2Body.prototype.GetAngularVelocity = funkcija () {
      vrni this.m_angularVelocity;
    };
    b2Body.prototype.GetDefinition = funkcija () {
      var bd = novo b2BodyDef();
      bd.type = this.GetType();
      bd.allowSleep =
        (this.m_flags & b2Body.e_allowSleepFlag) == b2Body.e_allowSleepFlag;
      bd.angle = this.GetAngle();
      bd.angularDamping = this.m_angularDamping;
      bd.angularVelocity = this.m_angularVelocity;
      bd.fixedRotation =
        (this.m_flags & b2Body.e_fixedRotationFlag) == b2Body.e_fixedRotationFlag;
      bd.bullet = (this.m_flags & b2Body.e_bulletFlag) == b2Body.e_bulletFlag;
      bd.awake = (this.m_flags & b2Body.e_awakeFlag) == b2Body.e_awakeFlag;
      bd.linearDamping = this.m_linearDamping;
      bd.linearVelocity.SetV(this.GetLinearVelocity());
      bd.position = this.GetPosition();
      bd.userData = this.GetUserData();
      vrnitev bd;
    };
    b2Body.prototype.ApplyForce = funkcija (sila, točka) {
      if (this.m_type != b2Body.b2_dynamicBody) {
        vrnitev;
      }
      if (this.IsAwake() == false) {
        this.SetAwake(true);
      }
      this.m_force.x += force.x;
      this.m_force.y += force.y;
      this.m_torque +=
        (point.x - this.m_sweep.cx) * force.y -
        (point.y - this.m_sweep.cy) * force.x;
    };
    b2Body.prototype.ApplyTorque = funkcija (navor) {
      če (navor === nedefiniran) navor = 0;
      if (this.m_type != b2Body.b2_dynamicBody) {
        vrnitev;
      }
      if (this.IsAwake() == false) {
        this.SetAwake(true);
      }
      this.m_torque += navor;
    };
    b2Body.prototype.ApplyImpulse = funkcija (impulz, točka) {
      if (this.m_type != b2Body.b2_dynamicBody) {
        vrnitev;
      }
      if (this.IsAwake() == false) {
        this.SetAwake(true);
      }
      this.m_linearVelocity.x += this.m_invMass * impulz.x;
      this.m_linearVelocity.y += this.m_invMass * impulz.y;
      this.m_angularVelocity +=
        this.m_invI *
        ((točka.x - this.m_sweep.cx) * impulz.y -
          (point.y - this.m_sweep.cy) * impulz.x);
    };
    b2Body.prototype.Split = funkcija (povratni klic) {
      var linearVelocity = this.GetLinearVelocity().Kopiraj();
      var angularVelocity = this.GetAngularVelocity();
      var center = this.GetWorldCenter();
      var telo1 = to;
      var body2 = this.m_world.CreateBody(this.GetDefinition());
      var prev;
      for (var f = body1.m_fixtureList; f; ) {
        if (povratni klic(f)) {
          var next = f.m_next;
          če (prejšnji) {
            prev.m_next = naslednji;
          } drugače {
            body1.m_fixtureList = naslednji;
          }
          body1.m_fixtureCount--;
          f.m_next = body2.m_fixtureList;
          body2.m_fixtureList = f;
          body2.m_fixtureCount++;
          f.m_telo = telo2;
          f = naslednji;
        } drugače {
          prejšnji = f;
          f = f.m_naslednji;
        }
      }
      body1.ResetMassData();
      body2.ResetMassData();
      var center1 = body1.GetWorldCenter();
      var center2 = body2.GetWorldCenter();
      var velocity1 = b2Math.AddVV(
        linearna hitrost,
        b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center1, center))
      );
      var velocity2 = b2Math.AddVV(
        linearna hitrost,
        b2Math.CrossFV(kotnaHitrost, b2Math.SubtractVV(center2, center))
      );
      body1.SetLinearVelocity(velocity1);
      body2.SetLinearVelocity(velocity2);
      body1.SetAngularVelocity(angularVelocity);
      body2.SetAngularVelocity(angularVelocity);
      body1.SynchronizeFixtures();
      body2.SynchronizeFixtures();
      vrni telo2;
    };
    b2Body.prototype.Merge = funkcija (drugo) {
      var f;
      for (f = other.m_fixtureList; f; ) {
        var next = f.m_next;
        other.m_fixtureCount--;
        f.m_next = this.m_fixtureList;
        this.m_fixtureList = f;
        this.m_fixtureCount++;
        f.m_telo = telo2;
        f = naslednji;
      }
      body1.m_fixtureCount = 0;
      var telo1 = to;
      var body2 = drugo;
      var center1 = body1.GetWorldCenter();
      var center2 = body2.GetWorldCenter();
      var velocity1 = body1.GetLinearVelocity().Kopiraj();
      var velocity2 = body2.GetLinearVelocity().Kopiraj();
      var angular1 = body1.GetAngularVelocity();
      var angular = body2.GetAngularVelocity();
      body1.ResetMassData();
      this.SynchronizeFixtures();
    };
    b2Body.prototype.GetMass = funkcija () {
      vrni to.m_mass;
    };
    b2Body.prototype.GetInertia = funkcija () {
      vrni to.m_I;
    };
    b2Body.prototype.GetMassData = funkcija (podatki) {
      data.mass = this.m_mass;
      data.I = this.m_I;
      data.center.SetV(this.m_sweep.localCenter);
    };
    b2Body.prototype.SetMassData = funkcija (massData) {
      b2Settings.b2Assert(this.m_world.IsLocked() == false);
      if (this.m_world.IsLocked() == true) {
        vrnitev;
      }
      if (this.m_type != b2Body.b2_dynamicBody) {
        vrnitev;
      }
      this.m_invMass = 0,0;
      this.m_I = 0,0;
      this.m_invI = 0,0;
      this.m_mass = massData.mass;
      if (this.m_mass <= 0,0) {
        this.m_masa = 1,0;
      }
      this.m_invMass = 1,0 / this.m_mass;
      if (massData.I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0) {
        to.m_jaz =
          massData.I -
          this.m_mass *
            (massData.center.x * massData.center.x +
              massData.center.y * massData.center.y);
        this.m_invI = 1,0 / this.m_I;
      }
      var oldCenter = this.m_sweep.c.Copy();
      this.m_sweep.localCenter.SetV(massData.center);
      this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
      this.m_sweep.c.SetV(this.m_sweep.c0);
      this.m_linearVelocity.x +=
        this.m_angularVelocity * -(this.m_sweep.cy - oldCenter.y);
      this.m_linearVelocity.y +=
        this.m_angularVelocity * +(this.m_sweep.cx - oldCenter.x);
    };
    b2Body.prototype.ResetMassData = funkcija () {
      this.m_masa = 0,0;
      this.m_invMass = 0,0;
      this.m_I = 0,0;
      this.m_invI = 0,0;
      this.m_sweep.localCenter.SetZero();
      če (
        this.m_type == b2Body.b2_staticBody ||
        this.m_type == b2Body.b2_kinematicBody
      ) {
        vrnitev;
      }
      var center = b2Vec2.Make(0, 0);
      for (var f = this.m_fixtureList; f; f = f.m_next) {
        if (f.m_density == 0,0) {
          nadaljevati;
        }
        var massData = f.GetMassData();
        this.m_mass += massData.mass;
        center.x += massData.center.x * massData.mass;
        center.y += massData.center.y * massData.mass;
        this.m_I += massData.I;
      }
      if (this.m_mass > 0,0) {
        this.m_invMass = 1,0 / this.m_mass;
        center.x *= this.m_invMass;
        center.y *= this.m_invMass;
      } drugače {
        this.m_masa = 1,0;
        this.m_invMass = 1,0;
      }
      if (this.m_I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0) {
        this.m_I -= this.m_mass * (center.x * center.x + center.y * center.y);
        this.m_I *= this.m_inertiaScale;
        b2Settings.b2Assert(this.m_I > 0);
        this.m_invI = 1,0 / this.m_I;
      } drugače {
        this.m_I = 0,0;
        this.m_invI = 0,0;
      }
      var oldCenter = this.m_sweep.c.Copy();
      this.m_sweep.localCenter.SetV(center);
      this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
      this.m_sweep.c.SetV(this.m_sweep.c0);
      this.m_linearVelocity.x +=
        this.m_angularVelocity * -(this.m_sweep.cy - oldCenter.y);
      this.m_linearVelocity.y +=
        this.m_angularVelocity * +(this.m_sweep.cx - oldCenter.x);
    };
    b2Body.prototype.GetWorldPoint = funkcija (localPoint) {
      var A = this.m_xf.R;
      var u = novo b2Vec2(
        A.col1.x * localPoint.x + A.col2.x * localPoint.y,
        A.col1.y * localPoint.x + A.col2.y * localPoint.y
      );
      ux += this.m_xf.position.x;
      uy += this.m_xf.position.y;
      vrni u;
    };
    b2Body.prototype.GetWorldVector = funkcija (localVector) {
      return b2Math.MulMV(this.m_xf.R, localVector);
    };
    b2Body.prototype.GetLocalPoint = funkcija (worldPoint) {
      return b2Math.MulXT(this.m_xf, worldPoint);
    };
    b2Body.prototype.GetLocalVector = funkcija (worldVector) {
      return b2Math.MulTMV(this.m_xf.R, worldVector);
    };
    b2Body.prototype.GetLinearVelocityFromWorldPoint = funkcija (worldPoint) {
      vrni novo b2Vec2(
        this.m_linearVelocity.x -
          this.m_angularVelocity * (worldPoint.y - this.m_sweep.cy),
        this.m_linearVelocity.y +
          this.m_angularVelocity * (worldPoint.x - this.m_sweep.cx)
      );
    };
    b2Body.prototype.GetLinearVelocityFromLocalPoint = funkcija (localPoint) {
      var A = this.m_xf.R;
      var worldPoint = novo b2Vec2(
        A.col1.x * localPoint.x + A.col2.x * localPoint.y,
        A.col1.y * localPoint.x + A.col2.y * localPoint.y
      );
      worldPoint.x += this.m_xf.position.x;
      worldPoint.y += this.m_xf.position.y;
      vrni novo b2Vec2(
        this.m_linearVelocity.x -
          this.m_angularVelocity * (worldPoint.y - this.m_sweep.cy),
        this.m_linearVelocity.y +
          this.m_angularVelocity * (worldPoint.x - this.m_sweep.cx)
      );
    };
    b2Body.prototype.GetLinearDamping = funkcija () {
      vrni this.m_linearDamping;
    };
    b2Body.prototype.SetLinearDamping = funkcija (linearDamping) {
      if (linearDamping === nedefinirano) linearDamping = 0;
      this.m_linearDamping = linearDamping;
    };
    b2Body.prototype.GetAngularDamping = funkcija () {
      vrni this.m_angularDamping;
    };
    b2Body.prototype.SetAngularDamping = funkcija (angularDamping) {
      če (kotno dušenje === nedefinirano) kotno dušenje = 0;
      this.m_angularDamping = kotno dušenje;
    };
    b2Body.prototype.SetType = funkcija (tip) {
      če (tip === nedefiniran) tip = 0;
      if (this.m_type == type) {
        vrnitev;
      }
      this.m_type = vrsta;
      this.ResetMassData();
      if (this.m_type == b2Body.b2_staticBody) {
        this.m_linearVelocity.SetZero();
        this.m_angularVelocity = 0,0;
      }
      this.SetAwake(true);
      this.m_force.SetZero();
      this.m_torque = 0,0;
      for (var ce = this.m_contactList; ce; ce = ce.next) {
        ce.contact.FlagForFiltering();
      }
    };
    b2Body.prototype.GetType = funkcija () {
      vrni this.m_type;
    };
    b2Body.prototype.SetBullet = funkcija (zastavica) {
      če (zastavica) {
        this.m_flags |= b2Body.e_bulletFlag;
      } drugače {
        this.m_flags &= ~b2Body.e_bulletFlag;
      }
    };
    b2Body.prototype.IsBullet = funkcija () {
      return (this.m_flags & b2Body.e_bulletFlag) == b2Body.e_bulletFlag;
    };
    b2Body.prototype.SetSleepingAllowed = funkcija (zastavica) {
      če (zastavica) {
        this.m_flags |= b2Body.e_allowSleepFlag;
      } drugače {
        this.m_flags &= ~b2Body.e_allowSleepFlag;
        this.SetAwake(true);
      }
    };
    b2Body.prototype.SetAwake = funkcija (zastavica) {
      če (zastavica) {
        this.m_flags |= b2Body.e_awakeFlag;
        this.m_sleepTime = 0,0;
      } drugače {
        this.m_flags &= ~b2Body.e_awakeFlag;
        this.m_sleepTime = 0,0;
        this.m_linearVelocity.SetZero();
        this.m_angularVelocity = 0,0;
        this.m_force.SetZero();
        this.m_torque = 0,0;
      }
    };
    b2Body.prototype.IsAwake = funkcija () {
      return (this.m_flags & b2Body.e_awakeFlag) == b2Body.e_awakeFlag;
    };
    b2Body.prototype.SetFixedRotation = funkcija (fiksno) {
      če (fiksno) {
        this.m_flags |= b2Body.e_fixedRotationFlag;
      } drugače {
        this.m_flags &= ~b2Body.e_fixedRotationFlag;
      }
      this.ResetMassData();
    };
    b2Body.prototype.IsFixedRotation = funkcija () {
      vrnitev (
        (this.m_flags & b2Body.e_fixedRotationFlag) == b2Body.e_fixedRotationFlag
      );
    };
    b2Body.prototype.SetActive = funkcija (zastavica) {
      if (zastavica == this.IsActive()) {
        vrnitev;
      }
      var broadPhase;
      var f;
      če (zastavica) {
        this.m_flags |= b2Body.e_activeFlag;
        broadPhase = this.m_world.m_contactManager.m_broadPhase;
        za (f = this.m_fixtureList; f; f = f.m_next) {
          f.CreateProxy(broadPhase, this.m_xf);
        }
      } drugače {
        this.m_flags &= ~b2Body.e_activeFlag;
        broadPhase = this.m_world.m_contactManager.m_broadPhase;
        za (f = this.m_fixtureList; f; f = f.m_next) {
          f.DestroyProxy(broadPhase);
        }
        var ce = this.m_contactList;
        medtem ko (ce) {
          var ce0 = ce;
          ce = ce.naslednji;
          this.m_world.m_contactManager.Destroy(ce0.contact);
        }
        this.m_contactList = null;
      }
    };
    b2Body.prototype.IsActive = funkcija () {
      return (this.m_flags & b2Body.e_activeFlag) == b2Body.e_activeFlag;
    };
    b2Body.prototype.IsSleepingAllowed = funkcija () {
      return (this.m_flags & b2Body.e_allowSleepFlag) == b2Body.e_allowSleepFlag;
    };
    b2Body.prototype.GetFixtureList = funkcija () {
      vrni this.m_fixtureList;
    };
    b2Body.prototype.GetJointList = funkcija () {
      vrni this.m_jointList;
    };
    b2Body.prototype.GetControllerList = funkcija () {
      vrni this.m_controllerList;
    };
    b2Body.prototype.GetContactList = funkcija () {
      vrni this.m_contactList;
    };
    b2Body.prototype.GetNext = funkcija () {
      vrni this.m_next;
    };
    b2Body.prototype.GetUserData = funkcija () {
      vrni this.m_userData;
    };
    b2Body.prototype.SetUserData = funkcija (podatki) {
      this.m_userData = podatki;
    };
    b2Body.prototype.GetWorld = funkcija () {
      vrni this.m_world;
    };
    b2Body.prototype.b2Body = funkcija (bd, svet) {
      this.m_flags = 0;
      if (bd.bullet) {
        this.m_flags |= b2Body.e_bulletFlag;
      }
      if (bd.fixedRotation) {
        this.m_flags |= b2Body.e_fixedRotationFlag;
      }
      if (bd.allowSleep) {
        this.m_flags |= b2Body.e_allowSleepFlag;
      }
      if (bd.awake) {
        this.m_flags |= b2Body.e_awakeFlag;
      }
      če (bd.aktivno) {
        this.m_flags |= b2Body.e_activeFlag;
      }
      this.m_world = svet;
      this.m_xf.position.SetV(bd.position);
      this.m_xf.R.Set(bd.angle);
      this.m_sweep.localCenter.SetZero();
      this.m_sweep.t0 = 1,0;
      this.m_sweep.a0 = this.m_sweep.a = bd.angle;
      var tMat = this.m_xf.R;
      var tVec = this.m_sweep.localCenter;
      this.m_sweep.cx = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
      this.m_sweep.cy = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
      this.m_sweep.cx += this.m_xf.position.x;
      this.m_sweep.cy += this.m_xf.position.y;
      this.m_sweep.c0.SetV(this.m_sweep.c);
      this.m_jointList = null;
      this.m_controllerList = null;
      this.m_contactList = null;
      this.m_controllerCount = 0;
      this.m_prev = null;
      this.m_next = null;
      this.m_linearVelocity.SetV(bd.linearVelocity);
      this.m_angularVelocity = bd.angularVelocity;
      this.m_linearDamping = bd.linearDamping;
      this.m_angularDamping = bd.angularDamping;
      this.m_force.Set(0,0, 0,0);
      this.m_torque = 0,0;
      this.m_sleepTime = 0,0;
      this.m_type = bd.type;
      if (this.m_type == b2Body.b2_dynamicBody) {
        this.m_masa = 1,0;
        this.m_invMass = 1,0;
      } drugače {
        this.m_masa = 0,0;
        this.m_invMass = 0,0;
      }
      this.m_I = 0,0;
      this.m_invI = 0,0;
      this.m_inertiaScale = bd.inertiaScale;
      this.m_userData = bd.userData;
      this.m_fixtureList = null;
      this.m_fixtureCount = 0;
    };
    b2Body.prototype.SynchronizeFixtures = funkcija () {
      var xf1 = b2Body.s_xf1;
      xf1.R.Set(this.m_sweep.a0);
      var tMat = xf1.R;
      var tVec = this.m_sweep.localCenter;
      xf1.položaj.x =
        this.m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      xf1.position.y =
        this.m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
      var f;
      var broadPhase = this.m_world.m_contactManager.m_broadPhase;
      za (f = this.m_fixtureList; f; f = f.m_next) {
        f.Sinhroniziraj(broadPhase, xf1, this.m_xf);
      }
    };
    b2Body.prototype.SynchronizeTransform = funkcija () {
      this.m_xf.R.Set(this.m_sweep.a);
      var tMat = this.m_xf.R;
      var tVec = this.m_sweep.localCenter;
      this.m_xf.position.x =
        this.m_sweep.cx - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
      this.m_xf.position.y =
        this.m_sweep.cy - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    };
    b2Body.prototype.ShouldCollide = funkcija (drugo) {
      če (
        this.m_type != b2Body.b2_dynamicBody &&
        other.m_type != b2Body.b2_dynamicBody
      ) {
        vrni false;
      }
      for (var jn = this.m_jointList; jn; jn = jn.next) {
        če (jn.other == other)
          if (jn.joint.m_collideConnected == false) {
            vrni false;
          }
      }
      vrni resnico;
    };
    b2Body.prototype.Advance = funkcija (t) {
      if (t === nedefinirano) t = 0;
      this.m_sweep.Advance(t);
      this.m_sweep.c.SetV(this.m_sweep.c0);
      this.m_sweep.a = this.m_sweep.a0;
      this.SynchronizeTransform();
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Dynamics.b2Body.s_xf1 = novo b2Transform();
      Box2D.Dynamics.b2Body.e_islandFlag = 0x0001;
      Box2D.Dynamics.b2Body.e_awakeFlag = 0x0002;
      Box2D.Dynamics.b2Body.e_allowSleepFlag = 0x0004;
      Box2D.Dynamics.b2Body.e_bulletFlag = 0x0008;
      Box2D.Dynamics.b2Body.e_fixedRotationFlag = 0x0010;
      Box2D.Dynamics.b2Body.e_activeFlag = 0x0020;
      Box2D.Dynamics.b2Body.b2_staticBody = 0;
      Box2D.Dynamics.b2Body.b2_kinematicBody = 1;
      Box2D.Dynamics.b2Body.b2_dynamicBody = 2;
    });
    b2BodyDef.b2BodyDef = funkcija () {
      this.position = new b2Vec2();
      this.linearVelocity = novo b2Vec2();
    };
    b2BodyDef.prototype.b2BodyDef = funkcija () {
      this.userData = null;
      this.position.Set(0,0, 0,0);
      this.angle = 0,0;
      this.linearVelocity.Set(0, 0);
      this.angularVelocity = 0,0;
      this.linearDamping = 0,0;
      this.angularDamping = 0,0;
      this.allowSleep = res;
      this.awake = res;
      this.fixedRotation = false;
      this.bullet = false;
      this.type = b2Body.b2_staticBody;
      this.active = res;
      this.inertiaScale = 1,0;
    };
    b2ContactFilter.b2ContactFilter = funkcija () {};
    b2ContactFilter.prototype.ShouldCollide = funkcija (fikstureA, fixtureB) {
      var filter1 = fixtureA.GetFilterData();
      var filter2 = fixtureB.GetFilterData();
      if (filter1.groupIndex == filter2.groupIndex && filter1.groupIndex != 0) {
        vrni filter1.groupIndex > 0;
      }
      var trk =
        (filter1.maskBits & filter2.categoryBits) != 0 &&
        (filter1.categoryBits & filter2.maskBits) != 0;
      povratni trk;
    };
    b2ContactFilter.prototype.RayCollide = funkcija (userData, stalnica) {
      if (!userData) vrne true;
      vrni to.ShouldCollide(
        primerek userDataof b2Fixture? uporabniški podatki: nič,
        napeljava
      );
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Dynamics.b2ContactFilter.b2_defaultFilter = novo b2ContactFilter();
    });
    b2ContactImpulse.b2ContactImpulse = funkcija () {
      this.normalImpulses = new Vector_a2j_Number(
        b2Settings.b2_maxManifoldPoints
      );
      this.tangentImpulses = new Vector_a2j_Number(
        b2Settings.b2_maxManifoldPoints
      );
    };
    b2ContactListener.b2ContactListener = funkcija () {};
    b2ContactListener.prototype.BeginContact = funkcija (stik) {};
    b2ContactListener.prototype.EndContact = funkcija (stik) {};
    b2ContactListener.prototype.PreSolve = funkcija (stik, oldManifold) {};
    b2ContactListener.prototype.PostSolve = funkcija (stik, impulz) {};
    Box2D.postDefs.push(funkcija () {
      Box2D.Dynamics.b2ContactListener.b2_defaultListener =
        nov b2ContactListener();
    });
    b2ContactManager.b2ContactManager = funkcija () {};
    b2ContactManager.prototype.b2ContactManager = funkcija () {
      this.m_world = null;
      this.m_contactCount = 0;
      this.m_contactFilter = b2ContactFilter.b2_defaultFilter;
      this.m_contactListener = b2ContactListener.b2_defaultListener;
      this.m_contactFactory = novo b2ContactFactory(this.m_allocator);
      this.m_broadPhase = novo b2DynamicTreeBroadPhase();
    };
    b2ContactManager.prototype.AddPair = funkcija (
      proxyUserDataA,
      proxyUserDataB
    ) {
      var fixtureA = proxyUserDataA instanceof b2Fixture? proxyUserDataA: nič;
      var fix tureB = proxyUserDataB instanceof b2Fixture? proxyUserDataB: nič;
      var bodyA = fixtureA.GetBody();
      var bodyB = fixtureB.GetBody();
      if (bodyA == bodyB) return;
      var edge = bodyB.GetContactList();
      medtem ko (rob) {
        if (edge.other == bodyA) {
          var fA = edge.contact.GetFixtureA();
          var fB = edge.contact.GetFixtureB();
          if (fA == stalnicaA && fB == stalnicaB) return;
          if (fA == napeljavaB && fB == napeljavaA) vrnitev;
        }
        rob = rob.naslednji;
      }
      if (bodyB.ShouldCollide(bodyA) == false) {
        vrnitev;
      }
      if (this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
        vrnitev;
      }
      var c = this.m_contactFactory.Create(fixtureA, fixtureB);
      fixtureA = c.GetFixtureA();
      fixtureB = c.GetFixtureB();
      teloA = napeljavaA.m_telo;
      teloB = napeljavaB.m_telo;
      c.m_prev = nič;
      c.m_next = this.m_world.m_contactList;
      if (this.m_world.m_contactList != null) {
        this.m_world.m_contactList.m_prev = c;
      }
      this.m_world.m_contactList = c;
      c.m_nodeA.contact = c;
      c.m_nodeA.other = bodyB;
      c.m_nodeA.prev = null;
      c.m_nodeA.next = bodyA.m_contactList;
      if (bodyA.m_contactList != null) {
        bodyA.m_contactList.prev = c.m_nodeA;
      }
      bodyA.m_contactList = c.m_nodeA;
      c.m_nodeB.contact = c;
      c.m_nodeB.other = bodyA;
      c.m_nodeB.prev = null;
      c.m_nodeB.next = bodyB.m_contactList;
      if (bodyB.m_contactList != null) {
        bodyB.m_contactList.prev = c.m_nodeB;
      }
      bodyB.m_contactList = c.m_nodeB;
      ++this.m_world.m_contactCount;
      vrnitev;
    };
    b2ContactManager.prototype.FindNewContacts = funkcija () {
      this.m_broadPhase.UpdatePairs(Box2D.generateCallback(this, this.AddPair));
    };
    b2ContactManager.prototype.Destroy = funkcija (c) {
      var fixtureA = c.GetFixtureA();
      var fixtureB = c.GetFixtureB();
      var bodyA = fixtureA.GetBody();
      var bodyB = fixtureB.GetBody();
      if (c.IsTouching()) {
        this.m_contactListener.EndContact(c);
      }
      if (c.m_prev) {
        c.m_prev.m_naslednji = c.m_naslednji;
      }
      if (c.m_next) {
        c.m_next.m_prev = c.m_prev;
      }
      if (c == this.m_world.m_contactList) {
        this.m_world.m_contactList = c.m_next;
      }
      if (c.m_nodeA.prev) {
        c.m_nodeA.prev.next = c.m_nodeA.next;
      }
      if (c.m_nodeA.next) {
        c.m_nodeA.next.prev = c.m_nodeA.prev;
      }
      if (c.m_nodeA == bodyA.m_contactList) {
        bodyA.m_contactList = c.m_nodeA.next;
      }
      if (c.m_nodeB.prev) {
        c.m_nodeB.prev.next = c.m_nodeB.next;
      }
      if (c.m_nodeB.next) {
        c.m_nodeB.next.prev = c.m_nodeB.prev;
      }
      if (c.m_nodeB == bodyB.m_contactList) {
        bodyB.m_contactList = c.m_nodeB.next;
      }
      this.m_contactFactory.Destroy(c);
      --this.m_contactCount;
    };
    b2ContactManager.prototype.Collide = funkcija () {
      var c = this.m_world.m_contactList;
      medtem ko (c) {
        var fixtureA = c.GetFixtureA();
        var fixtureB = c.GetFixtureB();
        var bodyA = fixtureA.GetBody();
        var bodyB = fixtureB.GetBody();
        if (bodyA.IsAwake() == false && bodyB.IsAwake() == false) {
          c = c.GetNext();
          nadaljevati;
        }
        if (c.m_flags & b2Contact.e_filterFlag) {
          if (bodyB.ShouldCollide(bodyA) == false) {
            var cNuke = c;
            c = cNuke.GetNext();
            this.Destroy(cNuke);
            nadaljevati;
          }
          if (this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
            cNuke = c;
            c = cNuke.GetNext();
            this.Destroy(cNuke);
            nadaljevati;
          }
          c.m_flags &= ~b2Contact.e_filterFlag;
        }
        var proxyA = fixtureA.m_proxy;
        var proxyB = fixtureB.m_proxy;
        var overlap = this.m_broadPhase.TestOverlap(proxyA, proxyB);
        if (prekrivanje == false) {
          cNuke = c;
          c = cNuke.GetNext();
          this.Destroy(cNuke);
          nadaljevati;
        }
        c.Update(this.m_contactListener);
        c = c.GetNext();
      }
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Dynamics.b2ContactManager.s_evalCP = novo b2ContactPoint();
    });
    b2DebugDraw.b2DebugDraw = funkcija () {};
    b2DebugDraw.prototype.b2DebugDraw = funkcija () {};
    b2DebugDraw.prototype.SetFlags = funkcija (zastavice) {
      če (zastavice === nedefinirano) zastavice = 0;
    };
    b2DebugDraw.prototype.GetFlags = funkcija () {};
    b2DebugDraw.prototype.AppendFlags = funkcija (zastavice) {
      če (zastavice === nedefinirano) zastavice = 0;
    };
    b2DebugDraw.prototype.ClearFlags = funkcija (zastavice) {
      če (zastavice === nedefinirano) zastavice = 0;
    };
    b2DebugDraw.prototype.SetSprite = funkcija (sprite) {};
    b2DebugDraw.prototype.GetSprite = funkcija () {};
    b2DebugDraw.prototype.SetDrawScale = funkcija (drawScale) {
      if (drawScale === nedefinirano) drawScale = 0;
    };
    b2DebugDraw.prototype.GetDrawScale = funkcija () {};
    b2DebugDraw.prototype.SetLineThickness = funkcija (lineThickness) {
      if (Debelina črte === nedefinirano) Debelina črte = 0;
    };
    b2DebugDraw.prototype.GetLineThickness = funkcija () {};
    b2DebugDraw.prototype.SetAlpha = funkcija (alfa) {
      če (alfa === nedefinirano) alfa = 0;
    };
    b2DebugDraw.prototype.GetAlpha = funkcija () {};
    b2DebugDraw.prototype.SetFillAlpha = funkcija (alfa) {
      če (alfa === nedefinirano) alfa = 0;
    };
    b2DebugDraw.prototype.GetFillAlpha = funkcija () {};
    b2DebugDraw.prototype.SetXFormScale = funkcija (xformScale) {
      if (xformScale === nedefinirano) xformScale = 0;
    };
    b2DebugDraw.prototype.GetXFormScale = funkcija () {};
    b2DebugDraw.prototype.DrawPolygon = funkcija (točke, vertexCount, barva) {
      če (Štetje vertex === nedefinirano) Število vertex = 0;
    };
    b2DebugDraw.prototype.DrawSolidPolygon = funkcija (
      oglišča,
      vertexCount,
      barva
    ) {
      če (Štetje vertex === nedefinirano) Število vertex = 0;
    };
    b2DebugDraw.prototype.DrawCircle = funkcija (središče, polmer, barva) {
      če (polmer === nedefinirano) radij = 0;
    };
    b2DebugDraw.prototype.DrawSolidCircle = funkcija (
      center,
      polmer,
      os,
      barva
    ) {
      če (polmer === nedefinirano) radij = 0;
    };
    b2DebugDraw.prototype.DrawSegment = funkcija (p1, p2, barva) {};
    b2DebugDraw.prototype.DrawTransform = funkcija (xf) {};
    Box2D.postDefs.push(funkcija () {
      Box2D.Dynamics.b2DebugDraw.e_shapeBit = 0x0001;
      Box2D.Dynamics.b2DebugDraw.e_jointBit = 0x0002;
      Box2D.Dynamics.b2DebugDraw.e_aabbBit = 0x0004;
      Box2D.Dynamics.b2DebugDraw.e_pairBit = 0x0008;
      Box2D.Dynamics.b2DebugDraw.e_centerOfMassBit = 0x0010;
      Box2D.Dynamics.b2DebugDraw.e_controllerBit = 0x0020;
    });
    b2DestructionListener.b2DestructionListener = funkcija () {};
    b2DestructionListener.prototype.SayGoodbyeJoint = funkcija (skup) {};
    b2DestructionListener.prototype.SayGoodbyeFixture = funkcija (fiksa) {};
    b2FilterData.b2FilterData = funkcija () {
      this.categoryBits = 0x0001;
      this.maskBits = 0xffff;
      this.groupIndex = 0;
    };
    b2FilterData.prototype.Copy = funkcija () {
      var copy = new b2FilterData();
      copy.categoryBits = this.categoryBits;
      copy.maskBits = this.maskBits;
      copy.groupIndex = this.groupIndex;
      povratni izvod;
    };
    b2Fixture.b2Fixture = funkcija () {
      this.m_filter = novo b2FilterData();
    };
    b2Fixture.prototype.GetType = funkcija () {
      vrni this.m_shape.GetType();
    };
    b2Fixture.prototype.GetShape = funkcija () {
      vrni this.m_shape;
    };
    b2Fixture.prototype.SetSensor = funkcija (senzor) {
      if (this.m_isSensor == senzor) return;
      this.m_isSensor = senzor;
      if (this.m_body == null) return;
      var edge = this.m_body.GetContactList();
      medtem ko (rob) {
        var contact = edge.contact;
        var fixtureA = contact.GetFixtureA();
        var fixtureB = contact.GetFixtureB();
        če (napeljavaA == to || napeljavaB == to)
          contact.SetSensor(fikstureA.IsSensor() || fixtureB.IsSensor());
        rob = rob.naslednji;
      }
    };
    b2Fixture.prototype.IsSensor = funkcija () {
      vrni this.m_isSensor;
    };
    b2Fixture.prototype.SetFilterData = funkcija (filter) {
      this.m_filter = filter.Copy();
      if (this.m_body) return;
      var edge = this.m_body.GetContactList();
      medtem ko (rob) {
        var contact = edge.contact;
        var fixtureA = contact.GetFixtureA();
        var fixtureB = contact.GetFixtureB();
        if (fixtureA == this || fixtureB == this) contact.FlagForFiltering();
        rob = rob.naslednji;
      }
    };
    b2Fixture.prototype.GetFilterData = funkcija () {
      vrni this.m_filter.Copy();
    };
    b2Fixture.prototype.GetBody = funkcija () {
      vrni to.m_telo;
    };
    b2Fixture.prototype.GetNext = funkcija () {
      vrni this.m_next;
    };
    b2Fixture.prototype.GetUserData = funkcija () {
      vrni this.m_userData;
    };
    b2Fixture.prototype.SetUserData = funkcija (podatki) {
      this.m_userData = podatki;
    };
    b2Fixture.prototype.TestPoint = funkcija (p) {
      vrni this.m_shape.TestPoint(this.m_body.GetTransform(), p);
    };
    b2Fixture.prototype.RayCast = funkcija (izhod, vhod) {
      vrni this.m_shape.RayCast(output, input, this.m_body.GetTransform());
    };
    b2Fixture.prototype.GetMassData = funkcija (massData) {
      if (massData === nedefinirano) massData = null;
      if (massData == null) {
        masovniPodatki = novi b2MassData();
      }
      this.m_shape.ComputeMass(massData, this.m_density);
      vrni množične podatke;
    };
    b2Fixture.prototype.SetDensity = funkcija (gostota) {
      if (gostota === nedefinirano) gostota = 0;
      this.m_density = gostota;
    };
    b2Fixture.prototype.GetDensity = funkcija () {
      vrni this.m_density;
    };
    b2Fixture.prototype.GetFriction = funkcija () {
      vrni this.m_friction;
    };
    b2Fixture.prototype.SetFriction = funkcija (trenje) {
      if (trenje === nedefinirano) trenje = 0;
      this.m_friction = trenje;
    };
    b2Fixture.prototype.GetRestitution = funkcija () {
      vrni to.m_restitucija;
    };
    b2Fixture.prototype.SetRestitution = funkcija (restitucija) {
      če (restitucija === nedefinirano) restitucija = 0;
      this.m_restitution = povrnitev;
    };
    b2Fixture.prototype.GetAABB = funkcija () {
      vrni to.m_aabb;
    };
    b2Fixture.prototype.b2Fixture = funkcija () {
      this.m_aabb = novo b2AABB();
      this.m_userData = null;
      this.m_body = null;
      this.m_next = null;
      this.m_shape = null;
      this.m_density = 0,0;
      this.m_friction = 0,0;
      this.m_restitution = 0,0;
    };
    b2Fixture.prototype.Create = funkcija (telo, xf, def) {
      this.m_userData = def.userData;
      this.m_friction = def.trenje;
      this.m_restitution = def.restitution;
      this.m_body = telo;
      this.m_next = null;
      this.m_filter = def.filter.Copy();
      this.m_isSensor = def.isSensor;
      this.m_shape = def.shape.Copy();
      this.m_density = def.density;
    };
    b2Fixture.prototype.Destroy = funkcija () {
      this.m_shape = null;
    };
    b2Fixture.prototype.CreateProxy = funkcija (broadPhase, xf) {
      this.m_shape.ComputeAABB(this.m_aabb, xf);
      this.m_proxy = broadPhase.CreateProxy(this.m_aabb, this);
    };
    b2Fixture.prototype.DestroyProxy = funkcija (broadPhase) {
      if (this.m_proxy == null) {
        vrnitev;
      }
      broadPhase.DestroyProxy(this.m_proxy);
      this.m_proxy = null;
    };
    b2Fixture.prototype.Synchronize = funkcija (
      broadPhase,
      transformiraj1,
      transformacija2
    ) {
      if (!this.m_proxy) return;
      var aabb1 = novo b2AABB();
      var aabb2 = novo b2AABB();
      this.m_shape.ComputeAABB(aabb1, transform1);
      this.m_shape.ComputeAABB(aabb2, transform2);
      this.m_aabb.Combine(aabb1, aabb2);
      var displacement = b2Math.SubtractVV(
        transform2.position,
        transformacija1.položaj
      );
      broadPhase.MoveProxy(this.m_proxy, this.m_aabb, premik);
    };
    b2FixtureDef.b2FixtureDef = funkcija () {
      this.filter = new b2FilterData();
    };
    b2FixtureDef.prototype.b2FixtureDef = funkcija () {
      this.shape = null;
      this.userData = null;
      to.trenje = 0,2;
      this.restitution = 0,0;
      this.density = 0,0;
      this.filter.categoryBits = 0x0001;
      this.filter.maskBits = 0xffff;
      this.filter.groupIndex = 0;
      this.isSensor = false;
    };
    b2Island.b2Island = funkcija () {};
    b2Island.prototype.b2Island = funkcija () {
      this.m_bodies = nov vektor();
      this.m_contacts = nov vektor();
      this.m_joints = nov vektor();
    };
    b2Island.prototype.Initialize = funkcija (
      telesna zmogljivost,
      kontaktna zmogljivost,
      jointCapacity,
      razdelilnik,
      poslušalec,
      contactSolver
    ) {
      if (bodyCapacity === nedefinirano) bodyCapacity = 0;
      if (contactCapacity === nedefinirano) contactCapacity = 0;
      if (jointCapacity === nedefinirano) jointCapacity = 0;
      var i = 0;
      this.m_bodyCapacity = bodyCapacity;
      this.m_contactCapacity = contactCapacity;
      this.m_jointCapacity = jointCapacity;
      this.m_bodyCount = 0;
      this.m_contactCount = 0;
      this.m_jointCount = 0;
      this.m_allocator = razdelilnik;
      this.m_listener = poslušalec;
      this.m_contactSolver = contactSolver;
      za (i = this.m_bodies.length; i < bodyCapacity; i++)
        this.m_bodies[i] = null;
      za (i = this.m_contacts.length; i < contactCapacity; i++)
        this.m_contacts[i] = null;
      za (i = this.m_joints.length; i < jointCapacity; i++)
        this.m_joints[i] = null;
    };
    b2Island.prototype.Clear = funkcija () {
      this.m_bodyCount = 0;
      this.m_contactCount = 0;
      this.m_jointCount = 0;
    };
    b2Island.prototype.Solve = funkcija (korak, gravitacija, allowSleep) {
      var i = 0;
      var j = 0;
      var b;
      var sklep;
      for (i = 0; i < this.m_bodyCount; ++i) {
        b = this.m_bodies[i];
        if (b.GetType() != b2Body.b2_dynamicBody) nadaljuje;
        b.m_linearVelocity.x += step.dt * (gravity.x + b.m_invMass * b.m_force.x);
        b.m_linearVelocity.y += step.dt * (gravity.y + b.m_invMass * b.m_force.y);
        b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;
        b.m_linearVelocity.Multiply(
          b2Math.Clamp(1.0 - step.dt * b.m_linearDamping, 0.0, 1.0)
        );
        b.m_angularVelocity *= b2Math.Clamp(
          1.0 - step.dt * b.m_angularDamping,
          0,0,
          1.0
        );
      }
      this.m_contactSolver.Initialize(
        korak,
        this.m_contacts,
        this.m_contactCount,
        this.m_allocator
      );
      var contactSolver = this.m_contactSolver;
      contactSolver.InitVelocityConstraints(korak);
      for (i = 0; i < this.m_jointCount; ++i) {
        joint = this.m_joints[i];
        joint.InitVelocityConstraints(korak);
      }
      for (i = 0; i < step.velocityIterations; ++i) {
        za (j = 0; j < this.m_jointCount; ++j) {
          joint = this.m_joints[j];
          joint.SolveVelocityConstraints(korak);
        }
        contactSolver.SolveVelocityConstraints();
      }
      for (i = 0; i < this.m_jointCount; ++i) {
        joint = this.m_joints[i];
        joint.FinalizeVelocityConstraints();
      }
      contactSolver.FinalizeVelocityConstraints();
      for (i = 0; i < this.m_bodyCount; ++i) {
        b = this.m_bodies[i];
        if (b.GetType() == b2Body.b2_staticBody) nadaljuje;
        var translationX = step.dt * b.m_linearVelocity.x;
        var translationY = step.dt * b.m_linearVelocity.y;
        če (
          prevodX * prevodX + prevodY * prevodY >
          b2Settings.b2_maxTranslationSquared
        ) {
          b.m_linearVelocity.Normalize();
          b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * step.inv_dt;
          b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * step.inv_dt;
        }
        var rotation = step.dt * b.m_angularVelocity;
        if (rotacija * vrtenje > b2Settings.b2_maxRotationSquared) {
          if (b.m_angularVelocity < 0,0) {
            b.m_angularVelocity = -b2Settings.b2_maxRotation * step.inv_dt;
          } drugače {
            b.m_angularVelocity = b2Settings.b2_maxRotation * step.inv_dt;
          }
        }
        b.m_sweep.c0.SetV(b.m_sweep.c);
        b.m_sweep.a0 = b.m_sweep.a;
        b.m_sweep.cx += step.dt * b.m_linearVelocity.x;
        b.m_sweep.cy += step.dt * b.m_linearVelocity.y;
        b.m_sweep.a += step.dt * b.m_angularVelocity;
        b.SynchronizeTransform();
      }
      for (i = 0; i < step.positionIterations; ++i) {
        var contactsOkay = contactSolver.SolvePositionConstraints(
          b2Settings.b2_contactBaumgarte
        );
        var jointsOkay = res;
        za (j = 0; j < this.m_jointCount; ++j) {
          joint = this.m_joints[j];
          var jointOkay = joint.SolvePositionConstraints(
            b2Settings.b2_contactBaumgarte
          );
          jointsOkay = jointsOkay && jointOkay;
        }
        if (contactsOkay && jointsOkay) {
          odmor;
        }
      }
      this.Report(contactSolver.m_constraints);
      if (allowSleep) {
        var minSleepTime = Number.MAX_VALUE;
        var linTolSqr =
          b2Settings.b2_linearSleepTolerance * b2Settings.b2_linearSleepTolerance;
        var angTolSqr =
          b2Settings.b2_angularSleepTolerance *
          b2Settings.b2_angularSleepTolerance;
        for (i = 0; i < this.m_bodyCount; ++i) {
          b = this.m_bodies[i];
          if (b.GetType() == b2Body.b2_staticBody) {
            nadaljevati;
          }
          if ((b.m_flags & b2Body.e_allowSleepFlag) == 0) {
            b.m_sleepTime = 0,0;
            minSleepTime = 0,0;
          }
          če (
            (b.m_flags & b2Body.e_allowSleepFlag) == 0 ||
            b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
            b2Math.Dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr
          ) {
            b.m_sleepTime = 0,0;
            minSleepTime = 0,0;
          } drugače {
            b.m_sleepTime += step.dt;
            minSleepTime = b2Math.Min(minSleepTime, b.m_sleepTime);
          }
        }
        if (minSleepTime >= b2Settings.b2_timeToSleep) {
          for (i = 0; i < this.m_bodyCount; ++i) {
            b = this.m_bodies[i];
            b.SetAwake(false);
          }
        }
      }
    };
    b2Island.prototype.SolveTOI = funkcija (podkorak) {
      var i = 0;
      var j = 0;
      this.m_contactSolver.Initialize(
        podkorak,
        this.m_contacts,
        this.m_contactCount,
        this.m_allocator
      );
      var contactSolver = this.m_contactSolver;
      for (i = 0; i < this.m_jointCount; ++i) {
        this.m_joints[i].InitVelocityConstraints(subStep);
      }
      for (i = 0; i < subStep.velocityIterations; ++i) {
        contactSolver.SolveVelocityConstraints();
        za (j = 0; j < this.m_jointCount; ++j) {
          this.m_joints[j].SolveVelocityConstraints(subStep);
        }
      }
      for (i = 0; i < this.m_bodyCount; ++i) {
        var b = this.m_bodies[i];
        if (b.GetType() == b2Body.b2_staticBody) nadaljuje;
        var translationX = subStep.dt * b.m_linearVelocity.x;
        var translationY = subStep.dt * b.m_linearVelocity.y;
        če (
          prevodX * prevodX + prevodY * prevodY >
          b2Settings.b2_maxTranslationSquared
        ) {
          b.m_linearVelocity.Normalize();
          b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * subStep.inv_dt;
          b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * subStep.inv_dt;
        }
        var rotation = subStep.dt * b.m_angularVelocity;
        if (rotacija * vrtenje > b2Settings.b2_maxRotationSquared) {
          if (b.m_angularVelocity < 0,0) {
            b.m_angularVelocity = -b2Settings.b2_maxRotation * subStep.inv_dt;
          } drugače {
            b.m_angularVelocity = b2Settings.b2_maxRotation * subStep.inv_dt;
          }
        }
        b.m_sweep.c0.SetV(b.m_sweep.c);
        b.m_sweep.a0 = b.m_sweep.a;
        b.m_sweep.cx += subStep.dt * b.m_linearVelocity.x;
        b.m_sweep.cy += subStep.dt * b.m_linearVelocity.y;
        b.m_sweep.a += subStep.dt * b.m_angularVelocity;
        b.SynchronizeTransform();
      }
      var k_toiBaumgarte = 0,75;
      for (i = 0; i < subStep.positionIterations; ++i) {
        var contactsOkay = contactSolver.SolvePositionConstraints(k_toiBaumgarte);
        var jointsOkay = res;
        za (j = 0; j < this.m_jointCount; ++j) {
          var jointOkay = this.m_joints[j].SolvePositionConstraints(
            b2Settings.b2_contactBaumgarte
          );
          jointsOkay = jointsOkay && jointOkay;
        }
        if (contactsOkay && jointsOkay) {
          odmor;
        }
      }
      this.Report(contactSolver.m_constraints);
    };
    b2Island.prototype.Report = funkcija (omejitve) {
      if (this.m_listener == null) {
        vrnitev;
      }
      for (var i = 0; i < this.m_contactCount; ++i) {
        var c = this.m_contacts[i];
        var cc = omejitve[i];
        for (var j = 0; j < cc.pointCount; ++j) {
          b2Island.s_impulse.normalImpulses[j] = cc.points[j].normalImpulse;
          b2Island.s_impulse.tangentImpulses[j] = cc.points[j].tangentImpulse;
        }
        this.m_listener.PostSolve(c, b2Island.s_impulse);
      }
    };
    b2Island.prototype.AddBody = funkcija (telo) {
      body.m_islandIndex = this.m_bodyCount;
      this.m_bodies[this.m_bodyCount++] = telo;
    };
    b2Island.prototype.AddContact = funkcija (stik) {
      this.m_contacts[this.m_contactCount++] = stik;
    };
    b2Island.prototype.AddJoint = funkcija (skup) {
      this.m_joints[this.m_jointCount++] = spoj;
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Dynamics.b2Island.s_impulse = novo b2ContactImpulse();
    });
    b2TimeStep.b2TimeStep = funkcija () {};
    b2TimeStep.prototype.Set = funkcija (korak) {
      this.dt = step.dt;
      this.inv_dt = step.inv_dt;
      this.positionIterations = step.positionIterations;
      this.velocityIterations = step.velocityIterations;
      this.warmStarting = step.warmStarting;
    };
    b2World.b2World = funkcija () {
      this.s_stack = nov vektor();
      this.m_contactManager = nov b2ContactManager();
      this.m_contactSolver = novo b2ContactSolver();
      this.m_island = new b2Island();
    };
    b2World.prototype.b2World = funkcija (gravitacija, doSleep) {
      this.m_destructionListener = null;
      this.m_debugDraw = null;
      this.m_bodyList = null;
      this.m_contactList = null;
      this.m_jointList = null;
      this.m_controllerList = null;
      this.m_bodyCount = 0;
      this.m_contactCount = 0;
      this.m_jointCount = 0;
      this.m_controllerCount = 0;
      b2World.m_warmStarting = res;
      b2World.m_continuousPhysics = res;
      this.m_allowSleep = doSleep;
      this.m_gravity = gravitacija;
      this.m_inv_dt0 = 0,0;
      this.m_contactManager.m_world = to;
      var bd = novo b2BodyDef();
      this.m_groundBody = this.CreateBody(bd);
    };
    b2World.prototype.SetDestructionListener = funkcija (poslušalnik) {
      this.m_destructionListener = poslušalec;
    };
    b2World.prototype.SetContactFilter = funkcija (filter) {
      this.m_contactManager.m_contactFilter = filter;
    };
    b2World.prototype.SetContactListener = funkcija (poslušalec) {
      this.m_contactManager.m_contactListener = poslušalec;
    };
    b2World.prototype.SetDebugDraw = funkcija (debugDraw) {
      this.m_debugDraw = debugDraw;
    };
    b2World.prototype.SetBroadPhase = funkcija (broadPhase) {
      var oldBroadPhase = this.m_contactManager.m_broadPhase;
      this.m_contactManager.m_broadPhase = broadPhase;
      for (var b = this.m_bodyList; b; b = b.m_next) {
        for (var f = b.m_fixtureList; f; f = f.m_next) {
          f.m_proxy = broadPhase.CreateProxy(
            oldBroadPhase.GetFatAABB(f.m_proxy),
            f
          );
        }
      }
    };
    b2World.prototype.Validate = funkcija () {
      this.m_contactManager.m_broadPhase.Validate();
    };
    b2World.prototype.GetProxyCount = funkcija () {
      vrni this.m_contactManager.m_broadPhase.GetProxyCount();
    };
    b2World.prototype.CreateBody = funkcija (def) {
      if (this.IsLocked() == true) {
        vrni nič;
      }
      var b = novo b2Body(def, to);
      b.m_prev = nič;
      b.m_next = this.m_bodyList;
      if (this.m_bodyList) {
        this.m_bodyList.m_prev = b;
      }
      this.m_bodyList = b;
      ++this.m_bodyCount;
      vrnitev b;
    };
    b2World.prototype.DestroyBody = funkcija (b) {
      if (this.IsLocked() == true) {
        vrnitev;
      }
      var jn = b.m_jointList;
      medtem ko (jn) {
        var jn0 = jn;
        jn = jn.naslednji;
        if (this.m_destructionListener) {
          this.m_destructionListener.SayGoodbyeJoint(jn0.joint);
        }
        this.DestroyJoint(jn0.joint);
      }
      var coe = b.m_controllerList;
      medtem ko (coe) {
        var coe0 = coe;
        coe = coe.nextController;
        coe0.controller.RemoveBody(b);
      }
      var ce = b.m_contactList;
      medtem ko (ce) {
        var ce0 = ce;
        ce = ce.naslednji;
        this.m_contactManager.Destroy(ce0.contact);
      }
      b.m_contactList = null;
      var f = b.m_fixtureList;
      medtem ko (f) {
        var f0 = f;
        f = f.m_naslednji;
        if (this.m_destructionListener) {
          this.m_destructionListener.SayGoodbyeFixture(f0);
        }
        f0.DestroyProxy(this.m_contactManager.m_broadPhase);
        f0.Uniči();
      }
      b.m_fixtureList = null;
      b.m_fixtureCount = 0;
      if (b.m_prev) {
        b.m_prev.m_naslednji = b.m_naslednji;
      }
      if (b.m_next) {
        b.m_next.m_prev = b.m_prev;
      }
      if (b == this.m_bodyList) {
        this.m_bodyList = b.m_next;
      }
      --this.m_bodyCount;
    };
    b2World.prototype.CreateJoint = funkcija (def) {
      var j = b2Joint.Create(def, null);
      j.m_prev = nič;
      j.m_next = this.m_jointList;
      if (this.m_jointList) {
        this.m_jointList.m_prev = j;
      }
      this.m_jointList = j;
      ++this.m_jointCount;
      j.m_edgeA.joint = j;
      j.m_edgeA.other = j.m_bodyB;
      j.m_edgeA.prev = null;
      j.m_edgeA.next = j.m_bodyA.m_jointList;
      if (j.m_bodyA.m_jointList) j.m_bodyA.m_jointList.prev = j.m_edgeA;
      j.m_bodyA.m_jointList = j.m_edgeA;
      j.m_edgeB.joint = j;
      j.m_edgeB.other = j.m_bodyA;
      j.m_edgeB.prev = null;
      j.m_edgeB.next = j.m_bodyB.m_jointList;
      if (j.m_bodyB.m_jointList) j.m_bodyB.m_jointList.prev = j.m_edgeB;
      j.m_bodyB.m_jointList = j.m_edgeB;
      var bodyA = def.bodyA;
      var bodyB = def.bodyB;
      if (def.collideConnected == false) {
        var edge = bodyB.GetContactList();
        medtem ko (rob) {
          if (edge.other == bodyA) {
            edge.contact.FlagForFiltering();
          }
          rob = rob.naslednji;
        }
      }
      vrnitev j;
    };
    b2World.prototype.DestroyJoint = funkcija (j) {
      var collideConnected = j.m_collideConnected;
      if (j.m_prev) {
        j.m_prev.m_naslednji = j.m_naslednji;
      }
      če (j.m_naslednji) {
        j.m_next.m_prej = j.m_prej;
      }
      if (j == this.m_jointList) {
        this.m_jointList = j.m_next;
      }
      var bodyA = j.m_bodyA;
      var bodyB = j.m_bodyB;
      bodyA.SetAwake(true);
      bodyB.SetAwake(true);
      if (j.m_edgeA.prev) {
        j.m_edgeA.prev.next = j.m_edgeA.next;
      }
      if (j.m_edgeA.next) {
        j.m_edgeA.next.prev = j.m_edgeA.prev;
      }
      if (j.m_edgeA == bodyA.m_jointList) {
        bodyA.m_jointList = j.m_edgeA.next;
      }
      j.m_edgeA.prev = null;
      j.m_edgeA.next = nič;
      if (j.m_edgeB.prev) {
        j.m_edgeB.prev.next = j.m_edgeB.next;
      }
      if (j.m_edgeB.next) {
        j.m_edgeB.next.prev = j.m_edgeB.prev;
      }
      if (j.m_edgeB == bodyB.m_jointList) {
        bodyB.m_jointList = j.m_edgeB.next;
      }
      j.m_edgeB.prev = null;
      j.m_edgeB.next = null;
      b2Joint.Destroy(j, null);
      --this.m_jointCount;
      if (collideConnected == false) {
        var edge = bodyB.GetContactList();
        medtem ko (rob) {
          if (edge.other == bodyA) {
            edge.contact.FlagForFiltering();
          }
          rob = rob.naslednji;
        }
      }
    };
    b2World.prototype.AddController = funkcija (c) {
      c.m_next = this.m_controllerList;
      c.m_prev = nič;
      this.m_controllerList = c;
      c.m_svet = to;
      this.m_controllerCount++;
      vrnitev c;
    };
    b2World.prototype.RemoveController = funkcija (c) {
      if (c.m_prev) c.m_prev.m_next = c.m_next;
      if (c.m_next) c.m_next.m_prev = c.m_prev;
      if (this.m_controllerList == c) this.m_controllerList = c.m_next;
      this.m_controllerCount--;
    };
    b2World.prototype.CreateController = funkcija (krmilnik) {
      if (controller.m_world != this)
        throw new Error("Krmilnik je lahko samo član enega sveta");
      controller.m_next = this.m_controllerList;
      controller.m_prev = null;
      if (this.m_controllerList) this.m_controllerList.m_prev = krmilnik;
      this.m_controllerList = krmilnik;
      ++this.m_controllerCount;
      controller.m_world = to;
      povratni krmilnik;
    };
    b2World.prototype.DestroyController = funkcija (krmilnik) {
      krmilnik.Počisti();
      if (controller.m_next) controller.m_next.m_prev = controller.m_prev;
      if (controller.m_prev) controller.m_prev.m_next = controller.m_next;
      če (krmilnik == this.m_controllerList)
        this.m_controllerList = controller.m_next;
      --this.m_controllerCount;
    };
    b2World.prototype.SetWarmStarting = funkcija (zastavica) {
      b2World.m_warmStarting = zastavica;
    };
    b2World.prototype.SetContinuousPhysics = funkcija (zastavica) {
      b2World.m_continuousPhysics = zastavica;
    };
    b2World.prototype.GetBodyCount = funkcija () {
      vrni this.m_bodyCount;
    };
    b2World.prototype.GetJointCount = funkcija () {
      vrni this.m_jointCount;
    };
    b2World.prototype.GetContactCount = funkcija () {
      vrni this.m_contactCount;
    };
    b2World.prototype.SetGravity = funkcija (gravitacija) {
      this.m_gravity = gravitacija;
    };
    b2World.prototype.GetGravity = funkcija () {
      vrni this.m_gravity;
    };
    b2World.prototype.GetGroundBody = funkcija () {
      vrni this.m_groundBody;
    };
    b2World.prototype.Step = funkcija (
      dt,
      velocityIterations,
      positionIterations
    ) {
      če (dt === nedefinirano) dt = 0;
      if (velocityIterations === nedefinirano) velocityIterations = 0;
      if (positionIterations === nedefinirano) positionIterations = 0;
      if (this.m_flags & b2World.e_newFixture) {
        this.m_contactManager.FindNewContacts();
        this.m_flags &= ~b2World.e_newFixture;
      }
      this.m_flags |= b2World.e_locked;
      var korak = b2World.s_timestep2;
      korak.dt = dt;
      step.velocityIterations = velocityIterations;
      step.positionIterations = positionIterations;
      če (dt > 0,0) {
        step.inv_dt = 1,0 / dt;
      } drugače {
        step.inv_dt = 0,0;
      }
      step.dtRatio = this.m_inv_dt0 * dt;
      step.warmStarting = b2World.m_warmStarting;
      this.m_contactManager.Collide();
      če (step.dt > 0,0) {
        this.Solve(korak);
      }
      if (b2World.m_continuousPhysics && step.dt > 0,0) {
        this.SolveTOI(korak);
      }
      če (step.dt > 0,0) {
        this.m_inv_dt0 = step.inv_dt;
      }
      this.m_flags &= ~b2World.e_locked;
    };
    b2World.prototype.ClearForces = funkcija () {
      for (var body = this.m_bodyList; body; body = body.m_next) {
        body.m_force.SetZero();
        body.m_torque = 0,0;
      }
    };
    b2World.prototype.DrawDebugData = funkcija () {
      if (this.m_debugDraw == null) {
        vrnitev;
      }
      this.m_debugDraw.m_sprite.graphics.clear();
      var flags = this.m_debugDraw.GetFlags();
      var i = 0;
      var b;
      var f;
      var s;
      var j;
      var bp;
      var invQ = novo b2Vec2();
      var x1 = novo b2Vec2();
      var x2 = novo b2Vec2();
      var xf;
      var b1 = novo b2AABB();
      var b2 = novo b2AABB();
      var vs = [novo b2Vec2(), novo b2Vec2(), novo b2Vec2(), novo b2Vec2()];
      var barva = nova b2Color(0, 0, 0);
      if (flags & b2DebugDraw.e_shapeBit) {
        for (b = this.m_bodyList; b; b = b.m_next) {
          xf = b.m_xf;
          for (f = b.GetFixtureList(); f; f = f.m_next) {
            s = f.GetShape();
            if (b.IsActive() == false) {
              color.Set(0,5, 0,5, 0,3);
              this.DrawShape(s, xf, color);
            } else if (b.GetType() == b2Body.b2_staticBody) {
              color.Set(0,5, 0,9, 0,5);
              this.DrawShape(s, xf, color);
            } else if (b.GetType() == b2Body.b2_kinematicBody) {
              color.Set(0,5, 0,5, 0,9);
              this.DrawShape(s, xf, color);
            } else if (b.IsAwake() == false) {
              color.Set(0,6, 0,6, 0,6);
              this.DrawShape(s, xf, color);
            } drugače {
              color.Set(0,9, 0,7, 0,7);
              this.DrawShape(s, xf, color);
            }
          }
        }
      }
      if (flags & b2DebugDraw.e_jointBit) {
        za (j = this.m_jointList; j; j = j.m_next) {
          this.DrawJoint(j);
        }
      }
      if (flags & b2DebugDraw.e_controllerBit) {
        for (var c = this.m_controllerList; c; c = c.m_next) {
          c.Draw(this.m_debugDraw);
        }
      }
      if (flags & b2DebugDraw.e_pairBit) {
        color.Set(0,3, 0,9, 0,9);
        za (
          var contact = this.m_contactManager.m_contactList;
          stik;
          kontakt = kontakt.GetNext()
        ) {
          var fixtureA = contact.GetFixtureA();
          var fixtureB = contact.GetFixtureB();
          var cA = fixtureA.GetAABB().GetCenter();
          var cB = fixtureB.GetAABB().GetCenter();
          this.m_debugDraw.DrawSegment(cA, cB, barva);
        }
      }
      if (flags & b2DebugDraw.e_aabbBit) {
        bp = this.m_contactManager.m_broadPhase;
        vs = [novo b2Vec2(), novo b2Vec2(), novo b2Vec2(), novo b2Vec2()];
        for (b = this.m_bodyList; b; b = b.GetNext()) {
          if (b.IsActive() == false) {
            nadaljevati;
          }
          for (f = b.GetFixtureList(); f; f = f.GetNext()) {
            var aabb = bp.GetFatAABB(f.m_proxy);
            vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
            vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
            vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
            vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);
            this.m_debugDraw.DrawPolygon(vs, 4, barva);
          }
        }
      }
      if (flags & b2DebugDraw.e_centerOfMassBit) {
        for (b = this.m_bodyList; b; b = b.m_next) {
          xf = b2World.s_xf;
          xf.R = b.m_xf.R;
          xf.position = b.GetWorldCenter();
          this.m_debugDraw.DrawTransform(xf);
        }
      }
    };
    b2World.prototype.QueryAABB = funkcija (povratni klic, aabb) {
      var __this = to;
      var broadPhase = __this.m_contactManager.m_broadPhase;

      funkcija WorldQueryWrapper(proxy) {
        vrni povratni klic(broadPhase.GetUserData(proxy));
      }
      broadPhase.Query(WorldQueryWrapper, aabb);
    };
    b2World.prototype.QueryShape = funkcija (povratni klic, oblika, transformacija) {
      var __this = to;
      if (transform === nedefinirano) transform = null;
      if (transform == null) {
        transform = novo b2Transform();
        transform.SetIdentity();
      }
      var broadPhase = __this.m_contactManager.m_broadPhase;

      funkcija WorldQueryWrapper(proxy) {
        var fixture =
          broadPhase.GetUserData(proxy) primerek b2Fixture
            ? broadPhase.GetUserData(proxy)
            : nič;
        če (
          b2Shape.TestOverlap(
            oblika,
            transformacija,
            fixture.GetShape(),
            fixture.GetBody().GetTransform()
          )
        )
          povratni klic (napeljava);
        vrni resnico;
      }
      var aabb = novo b2AABB();
      oblika.IzračunajAABB(aabb, transformiraj);
      broadPhase.Query(WorldQueryWrapper, aabb);
    };
    b2World.prototype.QueryPoint = funkcija (povratni klic, p) {
      var __this = to;
      var broadPhase = __this.m_contactManager.m_broadPhase;

      funkcija WorldQueryWrapper(proxy) {
        var fixture =
          broadPhase.GetUserData(proxy) primerek b2Fixture
            ? broadPhase.GetUserData(proxy)
            : nič;
        if (fikstura.TestPoint(p)) vrni povratni klic(fikstura);
        vrni resnico;
      }
      var aabb = novo b2AABB();
      aabb.lowerBound.Set(
        px - b2Settings.b2_linearSlop,
        py - b2Settings.b2_linearSlop
      );
      aabb.upperBound.Set(
        px + b2Settings.b2_linearSlop,
        py + b2Settings.b2_linearSlop
      );
      broadPhase.Query(WorldQueryWrapper, aabb);
    };
    b2World.prototype.RayCast = funkcija (povratni klic, točka1, točka2) {
      var __this = to;
      var broadPhase = __this.m_contactManager.m_broadPhase;
      var output = new b2RayCastOutput();

      funkcija RayCastWrapper(vhod, proxy) {
        var userData = broadPhase.GetUserData(proxy);
        var fixture = userData instanceof b2Fixture? uporabniški podatki: nič;
        var hit = fixture.RayCast(izhod, vnos);
        if (hit) {
          var fraction = output.fraction;
          var točka = novo b2Vec2(
            (1,0 - ulomek) * točka1.x + ulomek * točka2.x,
            (1,0 - ulomek) * točka1.y + ulomek * točka2.y
          );
          vrni povratni klic(fikstura, točka, izhod.normalno, ulomek);
        }
        vrni input.maxFraction;
      }
      var input = new b2RayCastInput(point1, point2);
      broadPhase.RayCast(RayCastWrapper, vnos);
    };
    b2World.prototype.RayCastOne = funkcija (točka1, točka2) {
      var __this = to;
      var rezultat;

      funkcija RayCastOneWrapper(fikstura, točka, normalno, ulomek) {
        če (ulomek === nedefinirano) ulomek = 0;
        rezultat = tekma;
        povratna frakcija;
      }
      __this.RayCast(RayCastOneWrapper, točka1, točka2);
      vrni rezultat;
    };
    b2World.prototype.RayCastAll = funkcija (točka1, točka2) {
      var __this = to;
      var rezultat = nov vektor();

      funkcija RayCastAllWrapper(fikstura, točka, normalno, ulomek) {
        če (ulomek === nedefinirano) ulomek = 0;
        rezultat[rezultat.dolžina] = stalnica;
        vrnitev 1;
      }
      __this.RayCast(RayCastAllWrapper, točka1, točka2);
      vrni rezultat;
    };
    b2World.prototype.GetBodyList = funkcija () {
      vrni this.m_bodyList;
    };
    b2World.prototype.GetJointList = funkcija () {
      vrni this.m_jointList;
    };
    b2World.prototype.GetContactList = funkcija () {
      vrni this.m_contactList;
    };
    b2World.prototype.IsLocked = funkcija () {
      return (this.m_flags & b2World.e_locked) > 0;
    };
    b2World.prototype.Solve = funkcija (korak) {
      var b;
      za (
        var controller = this.m_controllerList;
        krmilnik;
        krmilnik = krmilnik.m_naslednji
      ) {
        krmilnik.Korak(korak);
      }
      var otok = this.m_island;
      island.Initialize(
        this.m_bodyCount,
        this.m_contactCount,
        this.m_jointCount,
        nič,
        this.m_contactManager.m_contactListener,
        this.m_contactSolver
      );
      for (b = this.m_bodyList; b; b = b.m_next) {
        b.m_flags &= ~b2Body.e_islandFlag;
      }
      for (var c = this.m_contactList; c; c = c.m_next) {
        c.m_flags &= ~b2Contact.e_islandFlag;
      }
      za (var j = this.m_jointList; j; j = j.m_next) {
        j.m_islandFlag = false;
      }
      var stackSize = parseInt(this.m_bodyCount);
      var stack = this.s_stack;
      for (var seed = this.m_bodyList; seed; seed = seed.m_next) {
        if (seed.m_flags & b2Body.e_islandFlag) {
          nadaljevati;
        }
        if (seed.IsAwake() == false || seed.IsActive() == false) {
          nadaljevati;
        }
        if (seed.GetType() == b2Body.b2_staticBody) {
          nadaljevati;
        }
        otok.Počisti();
        var stackCount = 0;
        stack[stackCount++] = seme;
        seed.m_flags |= b2Body.e_islandFlag;
        medtem ko (stackCount > 0) {
          b = stack[--stackCount];
          island.AddBody(b);
          if (b.IsAwake() == false) {
            b.SetAwake(true);
          }
          if (b.GetType() == b2Body.b2_staticBody) {
            nadaljevati;
          }
          var drugo;
          for (var ce = b.m_contactList; ce; ce = ce.next) {
            if (ce.contact.m_flags & b2Contact.e_islandFlag) {
              nadaljevati;
            }
            če (
              ce.contact.IsSensor() == res ||
              ce.contact.IsEnabled() == false ||
              ce.contact.IsTouching() == false
            ) {
              nadaljevati;
            }
            island.AddContact(ce.contact);
            ce.contact.m_flags |= b2Contact.e_islandFlag;
            drugo = ce.drugo;
            if (other.m_flags & b2Body.e_islandFlag) {
              nadaljevati;
            }
            stack[stackCount++] = drugo;
            other.m_flags |= b2Body.e_islandFlag;
          }
          for (var jn = b.m_jointList; jn; jn = jn.next) {
            if (jn.joint.m_islandFlag == true) {
              nadaljevati;
            }
            drugo = jn.drugo;
            if (other.IsActive() == false) {
              nadaljevati;
            }
            island.AddJoint(jn.joint);
            jn.joint.m_islandFlag = res;
            if (other.m_flags & b2Body.e_islandFlag) {
              nadaljevati;
            }
            stack[stackCount++] = drugo;
            other.m_flags |= b2Body.e_islandFlag;
          }
        }
        island.Solve(step, this.m_gravity, this.m_allowSleep);
        for (var i = 0; i < island.m_bodyCount; ++i) {
          b = island.m_bodies[i];
          if (b.GetType() == b2Body.b2_staticBody) {
            b.m_flags &= ~b2Body.e_islandFlag;
          }
        }
      }
      for (i = 0; i < stack.length; ++i) {
        if (!stack[i]) break;
        stack[i] = null;
      }
      for (b = this.m_bodyList; b; b = b.m_next) {
        if (b.IsAwake() == false || b.IsActive() == false) {
          nadaljevati;
        }
        if (b.GetType() == b2Body.b2_staticBody) {
          nadaljevati;
        }
        b.SynchronizeFixtures();
      }
      this.m_contactManager.FindNewContacts();
    };
    b2World.prototype.SolveTOI = funkcija (korak) {
      var b;
      var fA;
      var fB;
      var bA;
      var bB;
      var cEdge;
      var j;
      var otok = this.m_island;
      island.Initialize(
        this.m_bodyCount,
        b2Settings.b2_maxTOIContactsPerIsland,
        b2Settings.b2_maxTOIJointsPerIsland,
        nič,
        this.m_contactManager.m_contactListener,
        this.m_contactSolver
      );
      var queue = b2World.s_queue;
      for (b = this.m_bodyList; b; b = b.m_next) {
        b.m_flags &= ~b2Body.e_islandFlag;
        b.m_sweep.t0 = 0,0;
      }
      var c;
      za (c = this.m_contactList; c; c = c.m_next) {
        c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
      }
      za (j = this.m_jointList; j; j = j.m_next) {
        j.m_islandFlag = false;
      }
      za (;;) {
        var minContact = null;
        var minTOI = 1,0;
        za (c = this.m_contactList; c; c = c.m_next) {
          če (
            c.IsSensor() == res ||
            c.IsEnabled() == false ||
            c.IsContinuous() == false
          ) {
            nadaljevati;
          }
          var toi = 1,0;
          if (c.m_flags & b2Contact.e_toiFlag) {
            toi = c.m_toi;
          } drugače {
            fA = c.m_pritrditevA;
            fB = c.m_fiks B;
            bA = fA.m_telo;
            bB = fB.m_telo;
            če (
              (bA.GetType() != b2Body.b2_dynamicBody || bA.IsAwake() == false) &&
              (bB.GetType() != b2Body.b2_dynamicBody || bB.IsAwake() == false)
            ) {
              nadaljevati;
            }
            var t0 = bA.m_sweep.t0;
            if (bA.m_sweep.t0 < bB.m_sweep.t0) {
              t0 = bB.m_sweep.t0;
              bA.m_sweep.Advance(t0);
            } else if (bB.m_sweep.t0 < bA.m_sweep.t0) {
              t0 = bA.m_sweep.t0;
              bB.m_sweep.Advance(t0);
            }
            toi = c.ComputeTOI(bA.m_sweep, bB.m_sweep);
            b2Settings.b2Assert(0,0 <= toi && toi <= 1,0);
            če (toi > 0,0 && toi < 1,0) {
              toi = (1,0 - toi) * t0 + toi;
              če (toi > 1) toi = 1;
            }
            c.m_toi = toi;
            c.m_flags |= b2Contact.e_toiFlag;
          }
          if (Number.MIN_VALUE < toi && toi < minTOI) {
            minStik = c;
            minTOI = toi;
          }
        }
        if (minContact == null || 1,0 - 100,0 * Number.MIN_VALUE < minTOI) {
          odmor;
        }
        fA = minContact.m_fixtureA;
        fB = minContact.m_fixtureB;
        bA = fA.m_telo;
        bB = fB.m_telo;
        b2World.s_backupA.Set(bA.m_sweep);
        b2World.s_backupB.Set(bB.m_sweep);
        bA.Vnaprej (minTOI);
        bB.Vnaprej (minTOI);
        minContact.Update(this.m_contactManager.m_contactListener);
        minContact.m_flags &= ~b2Contact.e_toiFlag;
        if (minContact.IsSensor() == true || minContact.IsEnabled() == false) {
          bA.m_sweep.Set(b2World.s_backupA);
          bB.m_sweep.Set(b2World.s_backupB);
          bA.SynchronizeTransform();
          bB.SynchronizeTransform();
          nadaljevati;
        }
        if (minContact.IsTouching() == false) {
          nadaljevati;
        }
        var seme = bA;
        if (seed.GetType() != b2Body.b2_dynamicBody) {
          seme = bB;
        }
        otok.Počisti();
        var queueStart = 0;
        var queueSize = 0;
        čakalna vrsta[queueStart + queueSize++] = seme;
        seed.m_flags |= b2Body.e_islandFlag;
        medtem ko (Velikost čakalne vrste > 0) {
          b = čakalna vrsta[queueStart++];
          --queueSize;
          island.AddBody(b);
          if (b.IsAwake() == false) {
            b.SetAwake(true);
          }
          if (b.GetType() != b2Body.b2_dynamicBody) {
            nadaljevati;
          }
          za (cEdge = b.m_contactList; cEdge; cEdge = cEdge.next) {
            if (island.m_contactCount == island.m_contactCapacity) {
              odmor;
            }
            if (cEdge.contact.m_flags & b2Contact.e_islandFlag) {
              nadaljevati;
            }
            če (
              cEdge.contact.IsSensor() == res ||
              cEdge.contact.IsEnabled() == false ||
              cEdge.contact.IsTouching() == false
            ) {
              nadaljevati;
            }
            island.AddContact(cEdge.contact);
            cEdge.contact.m_flags |= b2Contact.e_islandFlag;
            var other = cEdge.other;
            if (other.m_flags & b2Body.e_islandFlag) {
              nadaljevati;
            }
            if (other.GetType() != b2Body.b2_staticBody) {
              other.Advance(minTOI);
              other.SetAwake(true);
            }
            čakalna vrsta [queueStart + queueSize] = drugo;
            ++Velikost čakalne vrste;
            other.m_flags |= b2Body.e_islandFlag;
          }
          for (var jEdge = b.m_jointList; jEdge; jEdge = jEdge.next) {
            if (island.m_jointCount == island.m_jointCapacity) nadaljuje;
            if (jEdge.joint.m_islandFlag == true) nadaljuje;
            drugo = jEdge.other;
            if (other.IsActive() == false) {
              nadaljevati;
            }
            island.AddJoint(jEdge.joint);
            jEdge.joint.m_islandFlag = res;
            if (other.m_flags & b2Body.e_islandFlag) nadaljuje;
            if (other.GetType() != b2Body.b2_staticBody) {
              other.Advance(minTOI);
              other.SetAwake(true);
            }
            čakalna vrsta [queueStart + queueSize] = drugo;
            ++Velikost čakalne vrste;
            other.m_flags |= b2Body.e_islandFlag;
          }
        }
        var subStep = b2World.s_timestep;
        subStep.warmStarting = false;
        subStep.dt = (1,0 - minTOI) * korak.dt;
        subStep.inv_dt = 1,0 / subStep.dt;
        subStep.dtRatio = 0,0;
        subStep.velocityIterations = korak.velocityIterations;
        subStep.positionIterations = step.positionIterations;
        island.SolveTOI(subStep);
        var i = 0;
        for (i = 0; i < island.m_bodyCount; ++i) {
          b = island.m_bodies[i];
          b.m_flags &= ~b2Body.e_islandFlag;
          if (b.IsAwake() == false) {
            nadaljevati;
          }
          if (b.GetType() != b2Body.b2_dynamicBody) {
            nadaljevati;
          }
          b.SynchronizeFixtures();
          za (cEdge = b.m_contactList; cEdge; cEdge = cEdge.next) {
            cEdge.contact.m_flags &= ~b2Contact.e_toiFlag;
          }
        }
        for (i = 0; i < island.m_contactCount; ++i) {
          c = island.m_contacts[i];
          c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
        }
        for (i = 0; i < island.m_jointCount; ++i) {
          j = island.m_joints[i];
          j.m_islandFlag = false;
        }
        this.m_contactManager.FindNewContacts();
      }
    };
    b2World.prototype.DrawJoint = funkcija (skup) {
      var b1 = joint.GetBodyA();
      var b2 = joint.GetBodyB();
      var xf1 = b1.m_xf;
      var xf2 = b2.m_xf;
      var x1 = xf1.position;
      var x2 = xf2.position;
      var p1 = joint.GetAnchorA();
      var p2 = joint.GetAnchorB();
      var color = b2World.s_jointColor;
      stikalo (joint.m_type) {
        case b2Joint.e_distanceJoint:
          this.m_debugDraw.DrawSegment(p1, p2, barva);
          odmor;
        primer b2Joint.e_pulleyJoint:
          {
            var škripec = skupni primerek b2PulleyJoint? joint : nič;
            var s1 = škripec.GetGroundAnchorA();
            var s2 = škripec.GetGroundAnchorB();
            this.m_debugDraw.DrawSegment(s1, p1, barva);
            this.m_debugDraw.DrawSegment(s2, p2, barva);
            this.m_debugDraw.DrawSegment(s1, s2, barva);
          }
          odmor;
        case b2Joint.e_mouseJoint:
          this.m_debugDraw.DrawSegment(p1, p2, barva);
          odmor;
        privzeto:
          če (b1 != this.m_groundBody)
            this.m_debugDraw.DrawSegment(x1, p1, barva);
          this.m_debugDraw.DrawSegment(p1, p2, barva);
          če (b2 != this.m_groundBody)
            this.m_debugDraw.DrawSegment(x2, p2, barva);
      }
    };
    b2World.prototype.DrawShape = funkcija (oblika, xf, barva) {
      stikalo (shape.m_type) {
        case b2Shape.e_circleShape:
          {
            var circle = shape instanceof b2CircleShape? oblika : nič;
            var središče = b2Math.MulX(xf, krog.m_p);
            var radius = circle.m_radius;
            var os = xf.R.col1;
            this.m_debugDraw.DrawSolidCircle(središče, polmer, os, barva);
          }
          odmor;
        primer b2Shape.e_polygonShape:
          {
            var i = 0;
            var poly = shape instanceof b2PolygonShape? oblika : nič;
            var vertexCount = parseInt(poly.GetVertexCount());
            var localVertices = poly.GetVertices();
            var vertices = new Vector(vertexCount);
            for (i = 0; i < vertexCount; ++i) {
              oglišča[i] = b2Math.MulX(xf, lokalna oglišča[i]);
            }
            this.m_debugDraw.DrawSolidPolygon(točke, število točk, barva);
          }
          odmor;
        primeru b2Shape.e_edgeShape:
          {
            var edge = shape instanceof b2EdgeShape? oblika : nič;
            this.m_debugDraw.DrawSegment(
              b2Math.MulX(xf, edge.GetVertex1()),
              b2Math.MulX(xf, edge.GetVertex2()),
              barva
            );
          }
          odmor;
      }
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Dynamics.b2World.s_timestep2 = novo b2TimeStep();
      Box2D.Dynamics.b2World.s_xf = novo b2Transform();
      Box2D.Dynamics.b2World.s_backupA = novo b2Sweep();
      Box2D.Dynamics.b2World.s_backupB = novo b2Sweep();
      Box2D.Dynamics.b2World.s_timestep = novo b2TimeStep();
      Box2D.Dynamics.b2World.s_queue = nov vektor();
      Box2D.Dynamics.b2World.s_jointColor = nova b2Color(0,5, 0,8, 0,8);
      Box2D.Dynamics.b2World.e_newFixture = 0x0001;
      Box2D.Dynamics.b2World.e_locked = 0x0002;
    });
  })();
  (funkcija () {
    var b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
      b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef,
      b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape,
      b2MassData = Box2D.Collision.Shapes.b2MassData,
      b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
      b2Shape = Box2D.Collision.Shapes.b2Shape,
      b2CircleContact = Box2D.Dynamics.Contacts.b2CircleContact,
      b2Contact = Box2D.Dynamics.Contacts.b2Contact,
      b2ContactConstraint = Box2D.Dynamics.Contacts.b2ContactConstraint,
      b2ContactConstraintPoint = Box2D.Dynamics.Contacts.b2ContactConstraintPoint,
      b2ContactEdge = Box2D.Dynamics.Contacts.b2ContactEdge,
      b2ContactFactory = Box2D.Dynamics.Contacts.b2ContactFactory,
      b2ContactRegister = Box2D.Dynamics.Contacts.b2ContactRegister,
      b2ContactResult = Box2D.Dynamics.Contacts.b2ContactResult,
      b2ContactSolver = Box2D.Dynamics.Contacts.b2ContactSolver,
      b2EdgeAndCircleContact = Box2D.Dynamics.Contacts.b2EdgeAndCircleContact,
      b2NullContact = Box2D.Dynamics.Contacts.b2NullContact,
      b2PolyAndCircleContact = Box2D.Dynamics.Contacts.b2PolyAndCircleContact,
      b2PolyAndEdgeContact = Box2D.Dynamics.Contacts.b2PolyAndEdgeContact,
      b2PolygonContact = Box2D.Dynamics.Contacts.b2PolygonContact,
      b2PositionSolverManifold = Box2D.Dynamics.Contacts.b2PositionSolverManifold,
      b2Body = Box2D.Dynamics.b2Body,
      b2BodyDef = Box2D.Dynamics.b2BodyDef,
      b2ContactFilter = Box2D.Dynamics.b2ContactFilter,
      b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse,
      b2ContactListener = Box2D.Dynamics.b2ContactListener,
      b2ContactManager = Box2D.Dynamics.b2ContactManager,
      b2DebugDraw = Box2D.Dynamics.b2DebugDraw,
      b2DestructionListener = Box2D.Dynamics.b2DestructionListener,
      b2FilterData = Box2D.Dynamics.b2FilterData,
      b2Fixture = Box2D.Dynamics.b2Fixture,
      b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
      b2Island = Box2D.Dynamics.b2Island,
      b2TimeStep = Box2D.Dynamics.b2TimeStep,
      b2World = Box2D.Dynamics.b2World,
      b2Color = Box2D.Common.b2Color,
      b2notranji = Box2D.Common.b2notranji,
      b2Settings = Box2D.Common.b2Settings,
      b2Mat22 = Box2D.Common.Math.b2Mat22,
      b2Mat33 = Box2D.Common.Math.b2Mat33,
      b2Math = Box2D.Common.Math.b2Math,
      b2Sweep = Box2D.Common.Math.b2Sweep,
      b2Transform = Box2D.Common.Math.b2Transform,
      b2Vec2 = Box2D.Common.Math.b2Vec2,
      b2Vec3 = Box2D.Common.Math.b2Vec3,
      b2AABB = Box2D.Collision.b2AABB,
      b2Bound = Box2D.Collision.b2Bound,
      b2BoundValues ​​= Box2D.Collision.b2BoundValues,
      b2Collision = Box2D.Collision.b2Collision,
      b2ContactID = Box2D.Collision.b2ContactID,
      b2ContactPoint = Box2D.Collision.b2ContactPoint,
      b2Distance = Box2D.Collision.b2Distance,
      b2DistanceInput = Box2D.Collision.b2DistanceInput,
      b2DistanceOutput = Box2D.Collision.b2DistanceOutput,
      b2DistanceProxy = Box2D.Collision.b2DistanceProxy,
      b2DynamicTree = Box2D.Collision.b2DynamicTree,
      b2DynamicTreeBroadPhase = Box2D.Collision.b2DynamicTreeBroadPhase,
      b2DynamicTreeNode = Box2D.Collision.b2DynamicTreeNode,
      b2DynamicTreePair = Box2D.Collision.b2DynamicTreePair,
      b2Manifold = Box2D.Collision.b2Manifold,
      b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint,
      b2Point = Box2D.Collision.b2Point,
      b2RayCastInput = Box2D.Collision.b2RayCastInput,
      b2RayCastOutput = Box2D.Collision.b2RayCastOutput,
      b2Segment = Box2D.Collision.b2Segment,
      b2SeparationFunction = Box2D.Collision.b2SeparationFunction,
      b2Simplex = Box2D.Collision.b2Simplex,
      b2SimplexCache = Box2D.Collision.b2SimplexCache,
      b2SimplexVertex = Box2D.Collision.b2SimplexVertex,
      b2TimeOfImpact = Box2D.Collision.b2TimeOfImpact,
      b2TOIInput = Box2D.Collision.b2TOIInput,
      b2WorldManifold = Box2D.Collision.b2WorldManifold,
      ClipVertex = Box2D.Collision.ClipVertex,
      Lastnosti = Box2D.Collision.Features,
      IBroadPhase = Box2D.Collision.IBroadPhase;

    Box2D.inherit(b2CircleContact, Box2D.Dynamics.Contacts.b2Contact);
    b2CircleContact.prototype.__super =
      Box2D.Dynamics.Contacts.b2Contact.prototype;
    b2CircleContact.b2CircleContact = funkcija () {
      Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(to, argumenti);
    };
    b2CircleContact.Create = funkcija (dodeljevalnik) {
      vrni nov b2CircleContact();
    };
    b2CircleContact.Destroy = funkcija (stik, alokator) {};
    b2CircleContact.prototype.Reset = funkcija (fikstureA, fixtureB) {
      this.__super.Reset.call(this, fixtureA, fixtureB);
    };
    b2CircleContact.prototype.Evaluate = function () {
      var bA = this.m_fixtureA.GetBody();
      var bB = this.m_fixtureB.GetBody();
      b2Collision.CollideCircles(
        this.m_manifold,
        this.m_fixtureA.GetShape() primerek b2CircleShape
          ? this.m_fixtureA.GetShape()
          : nič,
        bA.m_xf,
        this.m_fixtureB.GetShape() primerek b2CircleShape
          ? this.m_fixtureB.GetShape()
          : nič,
        bB.m_xf
      );
    };
    b2Contact.b2Contact = funkcija () {
      this.m_nodeA = novo b2ContactEdge();
      this.m_nodeB = novo b2ContactEdge();
      this.m_manifold = novo b2Manifold();
      this.m_oldManifold = novo b2Manifold();
    };
    b2Contact.prototype.GetManifold = funkcija () {
      vrni this.m_manifold;
    };
    b2Contact.prototype.GetWorldManifold = funkcija (worldManifold) {
      var bodyA = this.m_fixtureA.GetBody();
      var bodyB = this.m_fixtureB.GetBody();
      var shapeA = this.m_fixtureA.GetShape();
      var shapeB = this.m_fixtureB.GetShape();
      worldManifold.Initialize(
        this.m_manifold,
        bodyA.GetTransform(),
        oblikaA.m_polmer,
        bodyB.GetTransform(),
        oblikaB.m_polmer
      );
    };
    b2Contact.prototype.IsTouching = funkcija () {
      vrnitev (
        (this.m_flags & b2Contact.e_touchingFlag) == b2Contact.e_touchingFlag
      );
    };
    b2Contact.prototype.IsContinuous = funkcija () {
      vrnitev (
        (this.m_flags & b2Contact.e_continuousFlag) == b2Contact.e_continuousFlag
      );
    };
    b2Contact.prototype.SetSensor = funkcija (senzor) {
      if (senzor) {
        this.m_flags |= b2Contact.e_sensorFlag;
      } drugače {
        this.m_flags &= ~b2Contact.e_sensorFlag;
      }
    };
    b2Contact.prototype.IsSensor = funkcija () {
      return (this.m_flags & b2Contact.e_sensorFlag) == b2Contact.e_sensorFlag;
    };
    b2Contact.prototype.SetEnabled = funkcija (zastavica) {
      če (zastavica) {
        this.m_flags |= b2Contact.e_enabledFlag;
      } drugače {
        this.m_flags &= ~b2Contact.e_enabledFlag;
      }
    };
    b2Contact.prototype.IsEnabled = funkcija () {
      return (this.m_flags & b2Contact.e_enabledFlag) == b2Contact.e_enabledFlag;
    };
    b2Contact.prototype.GetNext = funkcija () {
      vrni this.m_next;
    };
    b2Contact.prototype.GetFixtureA = funkcija () {
      vrni this.m_fixtureA;
    };
    b2Contact.prototype.GetFixtureB = funkcija () {
      vrni this.m_fixtureB;
    };
    b2Contact.prototype.FlagForFiltering = funkcija () {
      this.m_flags |= b2Contact.e_filterFlag;
    };
    b2Contact.prototype.b2Contact = funkcija () {};
    b2Contact.prototype.Reset = funkcija (fikstureA, fixtureB) {
      if (fikstureA === nedefinirano) fixtureA = nič;
      if (fikstureB === nedefinirano) fixtureB = null;
      this.m_flags = b2Contact.e_enabledFlag;
      if (!fixtureA || !fixtureB) {
        this.m_fixtureA = nič;
        this.m_fixtureB = nič;
        vrnitev;
      }
      if (fixtureA.IsSensor() || fixtureB.IsSensor()) {
        this.m_flags |= b2Contact.e_sensorFlag;
      }
      var bodyA = fixtureA.GetBody();
      var bodyB = fixtureB.GetBody();
      če (
        bodyA.GetType() != b2Body.b2_dynamicBody ||
        bodyA.IsBullet() ||
        bodyB.GetType() != b2Body.b2_dynamicBody ||
        bodyB.IsBullet()
      ) {
        this.m_flags |= b2Contact.e_continuousFlag;
      }
      this.m_fixtureA = napeljavaA;
      this.m_fixtureB = napeljavaB;
      this.m_manifold.m_pointCount = 0;
      this.m_prev = null;
      this.m_next = null;
      this.m_nodeA.contact = null;
      this.m_nodeA.prev = null;
      this.m_nodeA.next = null;
      this.m_nodeA.other = null;
      this.m_nodeB.contact = null;
      this.m_nodeB.prev = null;
      this.m_nodeB.next = null;
      this.m_nodeB.other = null;
    };
    b2Contact.prototype.Update = funkcija (poslušalec) {
      var tManifold = this.m_oldManifold;
      this.m_oldManifold = this.m_manifold;
      this.m_manifold = tManifold;
      this.m_flags |= b2Contact.e_enabledFlag;
      var touching = false;
      var wasTouching =
        (this.m_flags & b2Contact.e_touchingFlag) == b2Contact.e_touchingFlag;
      var bodyA = this.m_fixtureA.m_body;
      var bodyB = this.m_fixtureB.m_body;
      var aabbOverlap = this.m_fixtureA.m_aabb.TestOverlap(
        this.m_fixtureB.m_aabb
      );
      if (this.m_flags & b2Contact.e_sensorFlag) {
        if (aabbOverlap) {
          var shapeA = this.m_fixtureA.GetShape();
          var shapeB = this.m_fixtureB.GetShape();
          var xfA = bodyA.GetTransform();
          var xfB = bodyB.GetTransform();
          dotikanje = b2Shape.TestOverlap(shapeA, xfA, shapeB, xfB);
        }
        this.m_manifold.m_pointCount = 0;
      } drugače {
        če (
          bodyA.GetType() != b2Body.b2_dynamicBody ||
          bodyA.IsBullet() ||
          bodyB.GetType() != b2Body.b2_dynamicBody ||
          bodyB.IsBullet()
        ) {
          this.m_flags |= b2Contact.e_continuousFlag;
        } drugače {
          this.m_flags &= ~b2Contact.e_continuousFlag;
        }
        if (aabbOverlap) {
          this.Evaluate();
          dotik = this.m_manifold.m_pointCount > 0;
          for (var i = 0; i < this.m_manifold.m_pointCount; ++i) {
            var mp2 = this.m_manifold.m_points[i];
            mp2.m_normalni impulz = 0,0;
            mp2.m_tangentImpulse = 0,0;
            var id2 = mp2.m_id;
            for (var j = 0; j < this.m_oldManifold.m_pointCount; ++j) {
              var mp1 = this.m_oldManifold.m_points[j];
              if (mp1.m_id.key == id2.key) {
                mp2.m_normalImpulse = mp1.m_normalImpulse;
                mp2.m_tangentImpulse = mp1.m_tangentImpulse;
                odmor;
              }
            }
          }
        } drugače {
          this.m_manifold.m_pointCount = 0;
        }
        if (touching != wasTouching) {
          bodyA.SetAwake(true);
          bodyB.SetAwake(true);
        }
      }
      če (se dotika) {
        this.m_flags |= b2Contact.e_touchingFlag;
      } drugače {
        this.m_flags &= ~b2Contact.e_touchingFlag;
      }
      if (wasTouching == false && touching == true) {
        poslušalec.BeginContact(to);
      }
      if (wasTouching == true && touching == false) {
        poslušalec.EndContact(to);
      }
      if ((this.m_flags & b2Contact.e_sensorFlag) == 0) {
        poslušalec.PreSolve(this, this.m_oldManifold);
      }
    };
    b2Contact.prototype.Evaluate = funkcija () {};
    b2Contact.prototype.ComputeTOI = funkcija (sweepA, sweepB) {
      b2Contact.s_input.proxyA.Set(this.m_fixtureA.GetShape());
      b2Contact.s_input.proxyB.Set(this.m_fixtureB.GetShape());
      b2Contact.s_input.sweepA = sweepA;
      b2Contact.s_input.sweepB = sweepB;
      b2Contact.s_input.tolerance = b2Settings.b2_linearSlop;
      return b2TimeOfImpact.TimeOfImpact(b2Contact.s_input);
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Dynamics.Contacts.b2Contact.e_sensorFlag = 0x0001;
      Box2D.Dynamics.Contacts.b2Contact.e_continuousFlag = 0x0002;
      Box2D.Dynamics.Contacts.b2Contact.e_islandFlag = 0x0004;
      Box2D.Dynamics.Contacts.b2Contact.e_toiFlag = 0x0008;
      Box2D.Dynamics.Contacts.b2Contact.e_touchingFlag = 0x0010;
      Box2D.Dynamics.Contacts.b2Contact.e_enabledFlag = 0x0020;
      Box2D.Dynamics.Contacts.b2Contact.e_filterFlag = 0x0040;
      Box2D.Dynamics.Contacts.b2Contact.s_input = novo b2TOIInput();
    });
    b2ContactConstraint.b2ContactConstraint = funkcija () {
      this.localPlaneNormal = novo b2Vec2();
      this.localPoint = novo b2Vec2();
      this.normal = novo b2Vec2();
      this.normalMass = new b2Mat22();
      this.K = novo b2Mat22();
    };
    b2ContactConstraint.prototype.b2ContactConstraint = funkcija ( ) {
      this.points = nov vektor(b2Settings.b2_maxManifoldPoints);
      for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
        this.points[i] = new b2ContactConstraintPoint();
      }
    };
    b2ContactConstraintPoint.b2ContactConstraintPoint = funkcija () {
      this.localPoint = novo b2Vec2();
      this.rA = novo b2Vec2();
      this.rB = novo b2Vec2();
    };
    b2ContactEdge.b2ContactEdge = funkcija () {};
    b2ContactFactory.b2ContactFactory = funkcija () {};
    b2ContactFactory.prototype.b2ContactFactory = funkcija (dodeljevalnik) {
      this.m_allocator = razdelilnik;
      this.InitializeRegisters();
    };
    b2ContactFactory.prototype.AddType = funkcija (
      createFcn,
      uničiFcn,
      tip1,
      tip2
    ) {
      če (tip1 === nedefiniran) tip1 = 0;
      if (type2 === nedefinirano) type2 = 0;
      this.m_registers[type1][type2].createFcn = createFcn;
      this.m_registers[type1][type2].destroyFcn = destroyFcn;
      this.m_registers[type1][type2].primary = true;
      if (tip1 != tip2) {
        this.m_registers[type2][type1].createFcn = createFcn;
        this.m_registers[type2][type1].destroyFcn = destroyFcn;
        this.m_registers[type2][type1].primary = false;
      }
    };
    b2ContactFactory.prototype.InitializeRegisters = funkcija () {
      this.m_registers = nov vektor(b2Shape.e_shapeTypeCount);
      for (var i = 0; i < b2Shape.e_shapeTypeCount; i++) {
        this.m_registers[i] = nov vektor(b2Shape.e_shapeTypeCount);
        for (var j = 0; j < b2Shape.e_shapeTypeCount; j++) {
          this.m_registers[i][j] = new b2ContactRegister();
        }
      }
      this.AddType(
        b2CircleContact.Create,
        b2CircleContact.Destroy,
        b2Shape.e_circleShape,
        b2Shape.e_circleShape
      );
      this.AddType(
        b2PolyAndCircleContact.Create,
        b2PolyAndCircleContact.Destroy,
        b2Shape.e_polygonShape,
        b2Shape.e_circleShape
      );
      this.AddType(
        b2PolygonContact.Create,
        b2PolygonContact.Destroy,
        b2Shape.e_polygonShape,
        b2Shape.e_polygonShape
      );
      this.AddType(
        b2EdgeAndCircleContact.Create,
        b2EdgeAndCircleContact.Destroy,
        b2Shape.e_edgeShape,
        b2Shape.e_circleShape
      );
      this.AddType(
        b2PolyAndEdgeContact.Create,
        b2PolyAndEdgeContact.Destroy,
        b2Shape.e_polygonShape,
        b2Shape.e_edgeShape
      );
    };
    b2ContactFactory.prototype.Create = funkcija (fikstureA, fixtureB) {
      var type1 = parseInt(fikstureA.GetType());
      var type2 = parseInt(fixtureB.GetType());
      var reg = this.m_registers[type1][type2];
      var c;
      if (reg.pool) {
        c = reg.pool;
        reg.pool = c.m_next;
        reg.poolCount--;
        c.Reset(fikstureA, fixtureB);
        vrnitev c;
      }
      var createFcn = reg.createFcn;
      if (createFcn != null) {
        if (reg.primary) {
          c = createFcn(this.m_allocator);
          c.Reset(fikstureA, fixtureB);
          vrnitev c;
        } drugače {
          c = createFcn(this.m_allocator);
          c.Reset(fikstureB, fixtureA);
          vrnitev c;
        }
      } drugače {
        vrni nič;
      }
    };
    b2ContactFactory.prototype.Destroy = funkcija (kontakt) {
      if (contact.m_manifold.m_pointCount > 0) {
        contact.m_fixtureA.m_body.SetAwake(true);
        contact.m_fixtureB.m_body.SetAwake(true);
      }
      var type1 = parseInt(contact.m_fixtureA.GetType());
      var type2 = parseInt(contact.m_fixtureB.GetType());
      var reg = this.m_registers[type1][type2];
      če (true) {
        reg.poolCount++;
        contact.m_next = reg.pool;
        reg.pool = stik;
      }
      var destroyFcn = reg.destroyFcn;
      destroyFcn(stik, this.m_allocator);
    };
    b2ContactRegister.b2ContactRegister = funkcija () {};
    b2ContactResult.b2ContactResult = funkcija () {
      this.position = new b2Vec2();
      this.normal = novo b2Vec2();
      this.id = new b2ContactID();
    };
    b2ContactSolver.b2ContactSolver = funkcija () {
      this.m_step = new b2TimeStep();
      this.m_constraints = nov vektor();
    };
    b2ContactSolver.prototype.b2ContactSolver = funkcija () {};
    b2ContactSolver.prototype.Initialize = funkcija (
      korak,
      kontakti,
      contactcount,
      razdelilnik
    ) {
      if (contactCount === nedefinirano) contactCount = 0;
      var kontakt;
      this.m_step.Set(korak);
      this.m_allocator = razdelilnik;
      var i = 0;
      var tVec;
      var tMat;
      this.m_constraintCount = contactCount;
      medtem ko (this.m_constraints.length < this.m_constraintCount) {
        this.m_constraints[this.m_constraints.length] = novo b2ContactConstraint();
      }
      for (i = 0; i < contactCount; ++i) {
        stik = stiki[i];
        var fixtureA = contact.m_fixtureA;
        var fixtureB = contact.m_fixtureB;
        var shapeA = fixtureA.m_shape;
        var shapeB = fixtureB.m_shape;
        var radiusA = shapeA.m_radius;
        var radiusB = shapeB.m_radius;
        var bodyA = fixtureA.m_body;
        var bodyB = fixtureB.m_body;
        var razdelilnik = contact.GetManifold();
        var friction = b2Settings.b2MixFriction(
          fixtureA.GetFriction(),
          fixtureB.GetFriction()
        );
        var restitution = b2Settings.b2MixRestitution(
          fixtureA.GetRestitution(),
          fixtureB.GetRestitution()
        );
        var vAX = bodyA.m_linearVelocity.x;
        var vAY = bodyA.m_linearVelocity.y;
        var vBX = bodyB.m_linearVelocity.x;
        var vBY = bodyB.m_linearVelocity.y;
        var wA = bodyA.m_angularVelocity;
        var wB = bodyB.m_angularVelocity;
        b2Settings.b2Assert(manifold.m_pointCount > 0);
        b2ContactSolver.s_worldManifold.Initialize(
          razdelilnik,
          bodyA.m_xf,
          radij A,
          teloB.m_xf,
          polmerB
        );
        var normalX = b2ContactSolver.s_worldManifold.m_normal.x;
        var normalY = b2ContactSolver.s_worldManifold.m_normal.y;
        var cc = this.m_constraints[i];
        cc.bodyA = teloA;
        cc.bodyB = teloB;
        cc.manifold = razdelilnik;
        cc.normal.x = normalX;
        cc.normal.y = normalnoY;
        cc.pointCount = manifold.m_pointCount;
        cc.friction = trenje;
        cc.restitution = povrnitev;
        cc.localPlaneNormal.x = manifold.m_localPlaneNormal.x;
        cc.localPlaneNormal.y = manifold.m_localPlaneNormal.y;
        cc.localPoint.x = manifold.m_localPoint.x;
        cc.localPoint.y = manifold.m_localPoint.y;
        cc.polmer = polmerA + polmerB;
        cc.type = manifold.m_type;
        for (var k = 0; k < cc.pointCount; ++k) {
          var cp = manifold.m_points[k];
          var ccp = cc.points[k];
          ccp.normalImpulse = cp.m_normalImpulse;
          ccp.tangentImpulse = cp.m_tangentImpulse;
          ccp.localPoint.SetV(cp.m_localPoint);
          var rAX = (ccp.rA.x =
            b2ContactSolver.s_worldManifold.m_points[k].x - bodyA.m_sweep.cx);
          var rAY = (ccp.rA.y =
            b2ContactSolver.s_worldManifold.m_points[k].y - bodyA.m_sweep.cy);
          var rBX = (ccp.rB.x =
            b2ContactSolver.s_worldManifold.m_points[k].x - bodyB.m_sweep.cx);
          var rBY = (ccp.rB.y =
            b2ContactSolver.s_worldManifold.m_points[k].y - bodyB.m_sweep.cy);
          var rnA = rAX * normalY - rAY * normalX;
          var rnB = rBX * normalY - rBY * normalX;
          rnA *= rnA;
          rnB *= rnB;
          var kNormalno =
            bodyA.m_invMass +
            bodyB.m_invMass +
            bodyA.m_invI * rnA +
            bodyB.m_invI * rnB;
          ccp.normalMass = 1,0 / kNormalno;
          var kEqualized =
            bodyA.m_mass * bodyA.m_invMass + bodyB.m_mass * bodyB.m_invMass;
          kIzenačeno +=
            bodyA.m_masa * bodyA.m_invI * rnA + bodyB.m_masa * bodyB.m_invI * rnB;
          ccp.equalizedMass = 1,0 / kEqualized;
          var tangentX = normalY;
          var tangentY = -normalX;
          var rtA = rAX * tangentY - rAY * tangentX;
          var rtB = rBX * tangentY - rBY * tangentX;
          rtA *= rtA;
          rtB *= rtB;
          var kTangent =
            bodyA.m_invMass +
            bodyB.m_invMass +
            bodyA.m_invI * rtA +
            bodyB.m_invI * rtB;
          ccp.tangentMass = 1,0 / kTangent;
          ccp.velocityBias = 0,0;
          var tX = vBX + -wB * rBY - vAX - -wA * rAY;
          var tY = vBY + wB * rBX - vAY - wA * rAX;
          var vRel = cc.normal.x * tX + cc.normal.y * tY;
          if (vRel < -b2Settings.b2_velocityThreshold) {
            ccp.velocityBias += -cc.restitution * vRel;
          }
        }
        if (cc.pointCount == 2) {
          var ccp1 = cc.points[0];
          var ccp2 = cc.points[1];
          var invMassA = bodyA.m_invMass;
          var invIA = bodyA.m_invI;
          var invMassB = bodyB.m_invMass;
          var invIB = bodyB.m_invI;
          var rn1A = ccp1.rA.x * normalY - ccp1.rA.y * normalX;
          var rn1B = ccp1.rB.x * normalY - ccp1.rB.y * normalX;
          var rn2A = ccp2.rA.x * normalY - ccp2.rA.y * normalX;
          var rn2B = ccp2.rB.x * normalY - ccp2.rB.y * normalX;
          var k11 =
            invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
          var k22 =
            invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
          var k12 =
            invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;
          var k_maxConditionNumber = 100,0;
          if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
            cc.K.col1.Set(k11, k12);
            cc.K.col2.Set(k12, k22);
            cc.K.GetInverse(cc.normalMass);
          } drugače {
            cc.pointCount = 1;
          }
        }
      }
    };
    b2ContactSolver.prototype.InitVelocityConstraints = funkcija (korak) {
      var tVec;
      var tVec2;
      var tMat;
      for (var i = 0; i < this.m_constraintCount; ++i) {
        var c = this.m_constraints[i];
        var bodyA = c.bodyA;
        var bodyB = c.bodyB;
        var invMassA = bodyA.m_invMass;
        var invIA = bodyA.m_invI;
        var invMassB = bodyB.m_invMass;
        var invIB = bodyB.m_invI;
        var normalX = c.normal.x;
        var normalY = c.normal.y;
        var tangentX = normalY;
        var tangentY = -normalX;
        var tX = 0;
        var j = 0;
        var tCount = 0;
        if (step.warmStarting) {
          tCount = c.pointCount;
          for (j = 0; j < tCount; ++j) {
            var ccp = c.points[j];
            ccp.normalImpulse *= step.dtRatio;
            ccp.tangentImpulse *= step.dtRatio;
            var PX = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX;
            var PY = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY;
            bodyA.m_angularVelocity -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
            bodyA.m_linearVelocity.x -= invMassA * PX;
            bodyA.m_linearVelocity.y -= invMassA * PY;
            bodyB.m_angularVelocity += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
            bodyB.m_linearVelocity.x += invMassB * PX;
            bodyB.m_linearVelocity.y += invMassB * PY;
          }
        } drugače {
          tCount = c.pointCount;
          for (j = 0; j < tCount; ++j) {
            var ccp2 = c.points[j];
            ccp2.normalImpulse = 0,0;
            ccp2.tangentImpulse = 0,0;
          }
        }
      }
    };
    b2ContactSolver.prototype.SolveVelocityConstraints = funkcija () {
      var j = 0;
      var ccp;
      var rAX = 0;
      var rAY = 0;
      var rBX = 0;
      var rBY = 0;
      var dvX = 0;
      var dvY = 0;
      var vn = 0;
      var vt = 0;
      var lambda = 0;
      var maxFriction = 0;
      var newImpulse = 0;
      var PX = 0;
      var. PY = 0;
      var dX = 0;
      var dY = 0;
      var P1X = 0;
      var. P1Y = 0;
      var P2X = 0;
      var P2Y = 0;
      var tMat;
      var tVec;
      for (var i = 0; i < this.m_constraintCount; ++i) {
        var c = this.m_constraints[i];
        var bodyA = c.bodyA;
        var bodyB = c.bodyB;
        var wA = bodyA.m_angularVelocity;
        var wB = bodyB.m_angularVelocity;
        var vA = bodyA.m_linearVelocity;
        var vB = bodyB.m_linearVelocity;
        var invMassA = bodyA.m_invMass;
        var invIA = bodyA.m_invI;
        var invMassB = bodyB.m_invMass;
        var invIB = bodyB.m_invI;
        var normalX = c.normal.x;
        var normalY = c.normal.y;
        var tangentX = normalY;
        var tangentY = -normalX;
        var trenje = c.trenje;
        var tX = 0;
        for (j = 0; j < c.pointCount; j++) {
          ccp = c.točke[j];
          dvX = vB.x - wB * ccp.rB.y - vA.x + wA * ccp.rA.y;
          dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x;
          vt = dvX * tangentaX + dvY * tangentaY;
          lambda = ccp.tangentMass * -vt;
          maxFriction = trenje * ccp.normalImpulse;
          newImpulse = b2Math.Clamp(
            ccp.tangentImpulse + lambda,
            -maxFriction,
            maxFriction
          );
          lambda = nov impulz - ccp.tangentni impulz;
          PX = lambda * tangenta X;
          PY = lambda * tangentaY;
          vA.x -= invMassA * PX;
          vA.y -= invMassA * PY;
          wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
          vB.x += invMassB * PX;
          vB.y += invMassB * PY;
          wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
          ccp.tangentImpulse = novImpulz;
        }
        var tCount = parseInt(c.pointCount);
        if (c.pointCount == 1) {
          ccp = c.points[0];
          dvX = vB.x + -wB * ccp.rB.y - vA.x - -wA * ccp.rA.y;
          dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x;
          vn = dvX * normalX + dvY * normalY;
          lambda = -ccp.normalMass * (vn - ccp.velocityBias);
          novimpulz = ccp.normalni impulz + lambda;
          novimpulz = novimpulz > 0? nov impulz: 0,0;
          lambda = newImpulse - ccp.normalImpulse;
          PX = lambda * normalno X;
          PY = lambda * normalnoY;
          vA.x -= invMassA * PX;
          vA.y -= invMassA * PY;
          wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
          vB.x += invMassB * PX;
          vB.y += invMassB * PY;
          wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
          ccp.normalImpulse = novImpulz;
        } drugače {
          var cp1 = c.points[0];
          var cp2 = c.points[1];
          var aX = cp1.normalImpulse;
          var aY = cp2.normalImpulse;
          var dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
          var dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
          var dv2X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
          var dv2Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
          var vn1 = dv1X * normalX + dv1Y * normalY;
          var vn2 = dv2X * normalX + dv2Y * normalY;
          var bX = vn1 - cp1.velocityBias;
          var bY = vn2 - cp2.velocityBias;
          tMat = cK;
          bX -= tMat.col1.x * aX + tMat.col2.x * aY;
          bY -= tMat.col1.y * aX + tMat.col2.y * aY;
          var k_errorTol = 0,001;
          za (;;) {
            tMat = c.normalMass;
            var xX = -(tMat.col1.x * bX + tMat.col2.x * bY);
            var xY = -(tMat.col1.y * bX + tMat.col2.y * bY);
            če (xX >= 0,0 && xY >= 0,0) {
              dX = xX - aX;
              dY = xY - aY;
              P1X = dX * normalno X;
              P1Y = dX * normalnoY;
              P2X = dY * normalno X;
              P2Y = dY * normalnoY;
              vA.x -= invMassA * (P1X + P2X);
              vA.y -= invMassA * (P1Y + P2Y);
              wA -=
                invIA *
                (cp1.rA.x * P1Y -
                  cp1.rA.y * P1X +
                  cp2.rA.x * P2Y -
                  cp2.rA.y * P2X);
              vB.x += invMassB * (P1X + P2X);
              vB.y += invMassB * (P1Y + P2Y);
              wB +=
                invIB *
                (cp1.rB.x * P1Y -
                  cp1.rB.y * P1X +
                  cp2.rB.x * P2Y -
                  cp2.rB.y * P2X);
              cp1.normalImpulse = xX;
              cp2.normalImpulse = xY;
              odmor;
            }
            xX = -cp1.normalMass * bX;
            xY = 0,0;
            vn1 = 0,0;
            vn2 = cKcol1.y * xX + bY;
            če (xX >= 0,0 && vn2 >= 0,0) {
              dX = xX - aX;
              dY = xY - aY;
              P1X = dX * normalno X;
              P1Y = dX * normalnoY;
              P2X = dY * normalno X;
              P2Y = dY * normalnoY;
              vA.x -= invMassA * (P1X + P2X);
              vA.y -= invMassA * (P1Y + P2Y);
              wA -=
                invIA *
                (cp1.rA.x * P1Y -
                  cp1.rA.y * P1X +
                  cp2.rA.x * P2Y -
                  cp2.rA.y * P2X);
              vB.x += invMassB * (P1X + P2X);
              vB.y += invMassB * (P1Y + P2Y);
              wB +=
                invIB *
                (cp1.rB.x * P1Y -
                  cp1.rB.y * P1X +
                  cp2.rB.x * P2Y -
                  cp2.rB.y * P2X);
              cp1.normalImpulse = xX;
              cp2.normalImpulse = xY;
              odmor;
            }
            xX = 0,0;
            xY = -cp2.normalna masa * bY;
            vn1 = cKcol2.x * xY + bX;
            vn2 = 0,0;
            če (xY >= 0,0 && vn1 >= 0,0) {
              dX = xX - aX;
              dY = xY - aY;
              P1X = dX * normalno X;
              P1Y = dX * normalnoY;
              P2X = dY * normalno X;
              P2Y = dY * normalnoY;
              vA.x -= invMassA * (P1X + P2X);
              vA.y -= invMassA * (P1Y + P2Y);
              wA -=
                invIA *
                (cp1.rA.x * P1Y -
                  cp1.rA.y * P1X +
                  cp2.rA.x * P2Y -
                  cp2.rA.y * P2X);
              vB.x += invMassB * (P1X + P2X);
              vB.y += invMassB * (P1Y + P2Y);
              wB +=
                invIB *
                (cp1.rB.x * P1Y -
                  cp1.rB.y * P1X +
                  cp2.rB.x * P2Y -
                  cp2.rB.y * P2X);
              cp1.normalImpulse = xX;
              cp2.normalImpulse = xY;
              odmor;
            }
            xX = 0,0;
            xY = 0,0;
            vn1 = bX;
            vn2 = bY;
            če (vn1 >= 0,0 && vn2 >= 0,0) {
              dX = xX - aX;
              dY = xY - aY;
              P1X = dX * normalno X;
              P1Y = dX * normalnoY;
              P2X = dY * normalno X;
              P2Y = dY * normalnoY;
              vA.x -= invMassA * (P1X + P2X);
              vA.y -= invMassA * (P1Y + P2Y);
              wA -=
                invIA *
                (cp1.rA.x * P1Y -
                  cp1.rA.y * P1X +
                  cp2.rA.x * P2Y -
                  cp2.rA.y * P2X);
              vB.x += invMassB * (P1X + P2X);
              vB.y += invMassB * (P1Y + P2Y);
              wB +=
                invIB *
                (cp1.rB.x * P1Y -
                  cp1.rB.y * P1X +
                  cp2.rB.x * P2Y -
                  cp2.rB.y * P2X);
              cp1.normalImpulse = xX;
              cp2.normalImpulse = xY;
              odmor;
            }
            odmor;
          }
        }
        bodyA.m_angularVelocity = wA;
        bodyB.m_angularVelocity = wB;
      }
    };
    b2ContactSolver.prototype.FinalizeVelocityConstraints = funkcija () {
      for (var i = 0; i < this.m_constraintCount; ++i) {
        var c = this.m_constraints[i];
        var m = c.razdelilnik;
        for (var j = 0; j < c.pointCount; ++j) {
          var točka1 = m.m_točk[j];
          var point2 = c.points[j];
          point1.m_normalImpulse = point2.normalImpulse;
          point1.m_tangentImpulse = point2.tangentImpulse;
        }
      }
    };
    b2ContactSolver.prototype.SolvePositionConstraints = funkcija (baumgarte) {
      if (baumgarte === nedefinirano) baumgarte = 0;
      var minSeparation = 0,0;
      for (var i = 0; i < this.m_constraintCount; i++) {
        var c = this.m_constraints[i];
        var bodyA = c.bodyA;
        var bodyB = c.bodyB;
        var invMassA = bodyA.m_mass * bodyA.m_invMass;
        var invIA = bodyA.m_mass * bodyA.m_invI;
        var invMassB = bodyB.m_mass * bodyB.m_invMass;
        var invIB = bodyB.m_mass * bodyB.m_invI;
        b2ContactSolver.s_psm.Initialize(c);
        var normal = b2ContactSolver.s_psm.m_normal;
        for (var j = 0; j < c.pointCount; j++) {
          var ccp = c.points[j];
          var point = b2ContactSolver.s_psm.m_points[j];
          var separation = b2ContactSolver.s_psm.m_separations[j];
          var rAX = point.x - bodyA.m_sweep.cx;
          var rAY = point.y - bodyA.m_sweep.cy;
          var rBX = point.x - bodyB.m_sweep.cx;
          var rBY = point.y - bodyB.m_sweep.cy;
          minSeparation = minSeparation <separation? minSeparation : ločitev;
          var C = b2Math.Clamp(
            baumgarte * (ločitev + b2Settings.b2_linearSlop),
            -b2Settings.b2_maxLinearCorrection,
            0,0
          );
          var impulz = -ccp.equalizedMass * C;
          var PX = impulz * normal.x;
          var PY = impulz * normalno.y;
          bodyA.m_sweep.cx -= invMassA * PX;
          bodyA.m_sweep.cy -= invMassA * PY;
          bodyA.m_sweep.a -= invIA * (rAX * PY - rAY * PX);
          bodyA.SynchronizeTransform();
          bodyB.m_sweep.cx += invMassB * PX;
          bodyB.m_sweep.cy += invMassB * PY;
          bodyB.m_sweep.a += invIB * (rBX * PY - rBY * PX);
          bodyB.SynchronizeTransform();
        }
      }
      return minSeparation > -1,5 * b2Settings.b2_linearSlop;
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Dynamics.Contacts.b2ContactSolver.s_worldManifold =
        novo b2WorldManifold();
      Box2D.Dynamics.Contacts.b2ContactSolver.s_psm =
        novo b2PositionSolverManifold();
    });
    Box2D.inherit(b2EdgeAndCircleContact, Box2D.Dynamics.Contacts.b2Contact);
    b2EdgeAndCircleContact.prototype.__super =
      Box2D.Dynamics.Contacts.b2Contact.prototype;
    b2EdgeAndCircleContact.b2EdgeAndCircleContact = funkcija () {
      Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(to, argumenti);
    };
    b2EdgeAndCircleContact.Create = funkcija (dodeljevalnik) {
      vrni nov b2EdgeAndCircleContact();
    };
    b2EdgeAndCircleContact.Destroy = funkcija (stik, alokator) {};
    b2EdgeAndCircleContact.prototype.Reset = funkcija (fikstureA, fixtureB) {
      this.__super.Reset.call(this, fixtureA, fixtureB);
    };
    b2EdgeAndCircleContact.prototype.Evaluate = function () {
      var bA = this.m_fixtureA.GetBody();
      var bB = this.m_fixtureB.GetBody();
      this.b2CollideEdgeAndCircle(
        this.m_manifold,
        this.m_fixtureA.GetShape() primerek b2EdgeShape
          ? this.m_fixtureA.GetShape()
          : nič,
        bA.m_xf,
        this.m_fixtureB.GetShape() primerek b2CircleShape
          ? this.m_fixtureB.GetShape()
          : nič,
        bB.m_xf
      );
    };
    b2EdgeAndCircleContact.prototype.b2CollideEdgeAndCircle = funkcija (
      razdelilnik,
      rob,
      xf1,
      krog,
      xf2
    ) {};
    Box2D.inherit(b2NullContact, Box2D.Dynamics.Contacts.b2Contact);
    b2NullContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
    b2NullContact.b2NullContact = funkcija () {
      Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(to, argumenti);
    };
    b2NullContact.prototype.b2NullContact = funkcija () {
      this.__super.b2Contact.call(this);
    };
    b2NullContact.prototype.Evaluate = funkcija () {};
    Box2D.inherit(b2PolyAndCircleContact, Box2D.Dynamics.Contacts.b2Contact);
    b2PolyAndCircleContact.prototype.__super =
      Box2D.Dynamics.Contacts.b2Contact.prototype;
    b2PolyAndCircleContact.b2PolyAndCircleContact = funkcija () {
      Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(to, argumenti);
    };
    b2PolyAndCircleContact.Create = funkcija (dodeljevalnik) {
      vrni nov b2PolyAndCircleContact();
    };
    b2PolyAndCircleContact.Destroy = funkcija (stik, alokator) {};
    b2PolyAndCircleContact.prototype.Reset = funkcija (fikstureA, fixtureB) {
      this.__super.Reset.call(this, fixtureA, fixtureB);
      b2Settings.b2Assert(fixtureA.GetType() == b2Shape.e_polygonShape);
      b2Settings.b2Assert(fixtureB.GetType() == b2Shape.e_circleShape);
    };
    b2PolyAndCircleContact.prototype.Evaluate = function () {
      var bA = this.m_fixtureA.m_body;
      var bB = this.m_fixtureB.m_body;
      b2Collision.CollidePolygonAndCircle(
        this.m_manifold,
        this.m_fixtureA.GetShape() primerek b2PolygonShape
          ? this.m_fixtureA.GetShape()
          : nič,
        bA.m_xf,
        this.m_fixtureB.GetShape() primerek b2CircleShape
          ? this.m_fixtureB.GetShape()
          : nič,
        bB.m_xf
      );
    };
    Box2D.inherit(b2PolyAndEdgeContact, Box2D.Dynamics.Contacts.b2Contact);
    b2PolyAndEdgeContact.prototype.__super =
      Box2D.Dynamics.Contacts.b2Contact.prototype;
    b2PolyAndEdgeContact.b2PolyAndEdgeContact = funkcija () {
      Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(to, argumenti);
    };
    b2PolyAndEdgeContact.Create = funkcija (dodeljevalnik) {
      vrni nov b2PolyAndEdgeContact();
    };
    b2PolyAndEdgeContact.Destroy = funkcija (stik, alokator) {};
    b2PolyAndEdgeContact.prototype.Reset = funkcija (fikstureA, fixtureB) {
      this.__super.Reset.call(this, fixtureA, fixtureB);
      b2Settings.b2Assert(fixtureA.GetType() == b2Shape.e_polygonShape);
      b2Settings.b2Assert(fixtureB.GetType() == b2Shape.e_edgeShape);
    };
    b2PolyAndEdgeContact.prototype.Evaluate = function () {
      var bA = this.m_fixtureA.GetBody();
      var bB = this.m_fixtureB.GetBody();
      this.b2CollidePolyAndEdge(
        this.m_manifold,
        this.m_fixtureA.GetShape() primerek b2PolygonShape
          ? this.m_fixtureA.GetShape()
          : nič,
        bA.m_xf,
        this.m_fixtureB.GetShape() primerek b2EdgeShape
          ? this.m_fixtureB.GetShape()
          : nič,
        bB.m_xf
      );
    };
    b2PolyAndEdgeContact.prototype.b2CollidePolyAndEdge = funkcija (
      razdelilnik,
      poligon,
      xf1,
      rob,
      xf2
    ) {};
    Box2D.inherit(b2PolygonContact, Box2D.Dynamics.Contacts.b2Contact);
    b2PolygonContact.prototype.__super =
      Box2D.Dynamics.Contacts.b2Contact.prototype;
    b2PolygonContact.b2PolygonContact = funkcija () {
      Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(to, argumenti);
    };
    b2PolygonContact.Create = funkcija (dodeljevalnik) {
      vrni nov b2PolygonContact();
    };
    b2PolygonContact.Destroy = funkcija (stik, alokator) {};
    b2PolygonContact.prototype.Reset = funkcija (fikstureA, fixtureB) {
      this.__super.Reset.call(this, fixtureA, fixtureB);
    };
    b2PolygonContact.prototype.Evaluate = function () {
      var bA = this.m_fixtureA.GetBody();
      var bB = this.m_fixtureB.GetBody();
      b2Collision.CollidePolygons(
        this.m_manifold,
        this.m_fixtureA.GetShape() primerek b2PolygonShape
          ? this.m_fixtureA.GetShape()
          : nič,
        bA.m_xf,
        this.m_fixtureB.GetShape() primerek b2PolygonShape
          ? this.m_fixtureB.GetShape()
          : nič,
        bB.m_xf
      );
    };
    b2PositionSolverManifold.b2PositionSolverManifold = funkcija () {};
    b2PositionSolverManifold.prototype.b2PositionSolverManifold = funkcija () {
      this.m_normal = novo b2Vec2();
      this.m_separations = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints);
      this.m_points = nov vektor(b2Settings.b2_maxManifoldPoints);
      for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
        this.m_points[i] = new b2Vec2();
      }
    };
    b2PositionSolverManifold.prototype.Initialize = funkcija (cc) {
      b2Settings.b2Assert(cc.pointCount > 0);
      var i = 0;
      var clipPointX = 0;
      var clipPointY = 0;
      var tMat;
      var tVec;
      var planePointX = 0;
      var planePointY = 0;
      stikalo (cc.type) {
        case b2Manifold.e_circles:
          {
            tMat = cc.bodyA.m_xf.R;
            tVec = cc.localPoint;
            var pointAX =
              cc.bodyA.m_xf.position.x +
              (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            var pointAY =
              cc.bodyA.m_xf.position.y +
              (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            tMat = cc.bodyB.m_xf.R;
            tVec = cc.points[0].localPoint;
            var pointBX =
              cc.bodyB.m_xf.position.x +
              (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            var pointBY =
              cc.bodyB.m_xf.position.y +
              (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            var dX = točkaBX - točkaAX;
            var dY = točkaBY - točkaAY;
            var d2 = dX * dX + dY * dY;
            če (d2 > število.MIN_VALUE * število.MIN_VALUE) {
              var d = Math.sqrt(d2);
              this.m_normal.x = dX / d;
              this.m_normal.y = dY / d;
            } drugače {
              this.m_normal.x = 1,0;
              this.m_normal.y = 0,0;
            }
            this.m_points[0].x = 0,5 * (pointAX + pointBX);
            this.m_points[0].y = 0,5 * (pointAY + pointBY);
            this.m_separations[0] =
              dX * this.m_normal.x + dY * this.m_normal.y - cc.polmer;
          }
          odmor;
        primer b2Manifold.e_faceA:
          {
            tMat = cc.bodyA.m_xf.R;
            tVec = cc.localPlaneNormal;
            this.m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            this.m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tMat = cc.bodyA.m_xf.R;
            tVec = cc.localPoint;
            ravninaPointX =
              cc.bodyA.m_xf.position.x +
              (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            ravninaPointY =
              cc.bodyA.m_xf.position.y +
              (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            tMat = cc.bodyB.m_xf.R;
            for (i = 0; i < cc.pointCount; ++i) {
              tVec = cc.points[i].localPoint;
              clipPointX =
                cc.bodyB.m_xf.position.x +
                (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
              clipPointY =
                cc.bodyB.m_xf.position.y +
                (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
              this.m_separations[i] =
                (clipPointX - ravninaPointX) * this.m_normal.x +
                (clipPointY - ravninaPointY) * this.m_normal.y -
                cc.polmer;
              this.m_points[i].x = clipPointX;
              this.m_points[i].y = clipPointY;
            }
          }
          odmor;
        primer b2Manifold.e_faceB:
          {
            tMat = cc.bodyB.m_xf.R;
            tVec = cc.localPlaneNormal;
            this.m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            this.m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            tMat = cc.bodyB.m_xf.R;
            tVec = cc.localPoint;
            ravninaPointX =
              cc.bodyB.m_xf.position.x +
              (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            ravninaPointY =
              cc.bodyB.m_xf.position.y +
              (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            tMat = cc.bodyA.m_xf.R;
            for (i = 0; i < cc.pointCount; ++i) {
              tVec = cc.points[i].localPoint;
              clipPointX =
                cc.bodyA.m_xf.position.x +
                (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
              clipPointY =
                cc.bodyA.m_xf.position.y +
                (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
              this.m_separations[i] =
                (clipPointX - ravninaPointX) * this.m_normal.x +
                (clipPointY - ravninaPointY) * this.m_normal.y -
                cc.polmer;
              this.m_points[i].Set(clipPointX, clipPointY);
            }
            this.m_normal.x *= -1;
            this.m_normal.y *= -1;
          }
          odmor;
      }
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Dynamics.Contacts.b2PositionSolverManifold.circlePointA =
        novo b2Vec2();
      Box2D.Dynamics.Contacts.b2PositionSolverManifold.circlePointB =
        novo b2Vec2();
    });
  })();
  (funkcija () {
    var b2Body = Box2D.Dynamics.b2Body,
      b2BodyDef = Box2D.Dynamics.b2BodyDef,
      b2ContactFilter = Box2D.Dynamics.b2ContactFilter,
      b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse,
      b2ContactListener = Box2D.Dynamics.b2ContactListener,
      b2ContactManager = Box2D.Dynamics.b2ContactManager,
      b2DebugDraw = Box2D.Dynamics.b2DebugDraw,
      b2DestructionListener = Box2D.Dynamics.b2DestructionListener,
      b2FilterData = Box2D.Dynamics.b2FilterData,
      b2Fixture = Box2D.Dynamics.b2Fixture,
      b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
      b2Island = Box2D.Dynamics.b2Island,
      b2TimeStep = Box2D.Dynamics.b2TimeStep,
      b2World = Box2D.Dynamics.b2World,
      b2Mat22 = Box2D.Common.Math.b2Mat22,
      b2Mat33 = Box2D.Common.Math.b2Mat33,
      b2Math = Box2D.Common.Math.b2Math,
      b2Sweep = Box2D.Common.Math.b2Sweep,
      b2Transform = Box2D.Common.Math.b2Transform,
      b2Vec2 = Box2D.Common.Math.b2Vec2,
      b2Vec3 = Box2D.Common.Math.b2Vec3,
      b2Color = Box2D.Common.b2Color,
      b2notranji = Box2D.Common.b2notranji,
      b2Settings = Box2D.Common.b2Settings,
      b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
      b2EdgeChainDef = Box2D.Collision.Shapes.b2EdgeChainDef,
      b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape,
      b2MassData = Box2D.Collision.Shapes.b2MassData,
      b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
      b2Shape = Box2D.Collision.Shapes.b2Shape,
      b2BuoyancyController = Box2D.Dynamics.Controllers.b2BuoyancyController,
      b2ConstantAccelController =
        Box2D.Dynamics.Controllers.b2ConstantAccelController,
      b2ConstantForceController =
        Box2D.Dynamics.Controllers.b2ConstantForceController,
      b2Controller = Box2D.Dynamics.Controllers.b2Controller,
      b2ControllerEdge = Box2D.Dynamics.Controllers.b2ControllerEdge,
      b2GravityController = Box2D.Dynamics.Controllers.b2GravityController,
      b2TensorDampingController =
        Box2D.Dynamics.Controllers.b2TensorDampingController;

    Box2D.inherit(b2BuoyancyController, Box2D.Dynamics.Controllers.b2Controller);
    b2BuoyancyController.prototype.__super =
      Box2D.Dynamics.Controllers.b2Controller.prototype;
    b2BuoyancyController.b2BuoyancyController = funkcija () {
      Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply(this, arguments);
      this.normal = novo b2Vec2(0, -1);
      this.offset = 0;
      this.density = 0;
      this.velocity = novo b2Vec2(0, 0);
      this.linearDrag = 2;
      this.angularDrag = 1;
      this.useDensity = false;
      this.useWorldGravity = res;
      this.gravity = null;
    };
    b2BuoyancyController.prototype.Step = funkcija (korak) {
      if (!this.m_bodyList) return;
      if (this.useWorldGravity) {
        this.gravity = this.GetWorld().GetGravity().Copy();
      }
      for (var i = this.m_bodyList; i; i = i.nextBody) {
        var body = i.body;
        if (body.IsAwake() == false) {
          nadaljevati;
        }
        var areac = novo b2Vec2();
        var massc = novo b2Vec2();
        var površina = 0,0;
        var masa = 0,0;
        za (
          var fixture = body.GetFixtureList();
          napeljava;
          fixture = fixture.GetNext()
        ) {
          var sc = novo b2Vec2();
          var sarea = stalnica
            .GetShape()
            .ComputeSubmergedArea(
              to.normalno,
              this.offset,
              body.GetTransform(),
              sc
            );
          območje += sarea;
          areac.x += sarea * sc.x;
          areac.y += sarea * sc.y;
          var shapeDensity = 0;
          if (this.useDensity) {
            Gostota oblike = 1;
          } drugače {
            Gostota oblike = 1;
          }
          masa += sarea * oblika Gostota;
          massc.x += sarea * sc.x * shapeDensity;
          massc.y += sarea * sc.y * shapeDensity;
        }
        areac.x /= območje;
        areac.y /= območje;
        massc.x /= masa;
        massc.y /= masa;
        if (area < Number.MIN_VALUE) continue;
        var buoyancyForce = this.gravity.GetNegative();
        plovnostForce.Multiply(this.density * area);
        body.ApplyForce(buoyancyForce, massc);
        var dragForce = body.GetLinearVelocityFromWorldPoint(areac);
        dragForce.Subtract(this.velocity);
        dragForce.Multiply(-this.linearDrag * območje);
        body.ApplyForce(dragForce, areac);
        body.ApplyTorque(
          (-body.GetInertia() / body.GetMass()) *
            območje *
            body.GetAngularVelocity() *
            this.angularDrag
        );
      }
    };
    b2BuoyancyController.prototype.Draw = funkcija (debugDraw) {
      var r = 1000;
      var p1 = novo b2Vec2();
      var p2 = novo b2Vec2();
      p1.x = this.normal.x * this.offset + this.normal.y * r;
      p1.y = this.normal.y * this.offset - this.normal.x * r;
      p2.x = this.normal.x * this.offset - this.normal.y * r;
      p2.y = this.normal.y * this.offset + this.normal.x * r;
      var barva = nova b2Color(0, 0, 1);
      debugDraw.DrawSegment(p1, p2, barva);
    };
    Box2D.inherit(
      b2ConstantAccelController,
      Box2D.Dynamics.Controllers.b2Controller
    );
    b2ConstantAccelController.prototype.__super =
      Box2D.Dynamics.Controllers.b2Controller.prototype;
    b2ConstantAccelController.b2ConstantAccelController = funkcija () {
      Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply(this, arguments);
      this.A = novo b2Vec2(0, 0);
    };
    b2ConstantAccelController.prototype.Step = funkcija (korak) {
      var smallA = new b2Vec2(this.Ax * step.dt, this.Ay * step.dt);
      for (var i = this.m_bodyList; i; i = i.nextBody) {
        var body = i.body;
        if (!body.IsAwake()) continue;
        body.SetLinearVelocity(
          novo b2Vec2(
            body.GetLinearVelocity().x + smallA.x,
            body.GetLinearVelocity().y + smallA.y
          )
        );
      }
    };
    Box2D.inherit(
      b2ConstantForceController,
      Box2D.Dynamics.Controllers.b2Controller
    );
    b2ConstantForceController.prototype.__super =
      Box2D.Dynamics.Controllers.b2Controller.prototype;
    b2ConstantForceController.b2ConstantForceController = funkcija () {
      Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply(this, arguments);
      this.F = novo b2Vec2(0, 0);
    };
    b2ConstantForceController.prototype.Step = funkcija (korak) {
      for (var i = this.m_bodyList; i; i = i.nextBody) {
        var body = i.body;
        if (!body.IsAwake()) continue;
        body.ApplyForce(this.F, body.GetWorldCenter());
      }
    };
    b2Controller.b2Controller = funkcija () {};
    b2Controller.prototype.Step = funkcija (korak) {};
    b2Controller.prototype.Draw = funkcija (debugDraw) {};
    b2Controller.prototype.AddBody = funkcija (telo) {
      var edge = new b2ControllerEdge();
      edge.controller = to;
      edge.body = telo;
      edge.nextBody = this.m_bodyList;
      edge.prevBody = null;
      this.m_bodyList = rob;
      if (edge.nextBody) edge.nextBody.prevBody = rob;
      this.m_bodyCount++;
      edge.nextController = body.m_controllerList;
      edge.prevController = null;
      body.m_controllerList = rob;
      if (edge.nextController) edge.nextController.prevController = rob;
      body.m_controllerCount++;
    };
    b2Controller.prototype.RemoveBody = funkcija (telo) {
      var edge = body.m_controllerList;
      medtem ko (edge ​​&& edge.controller != this) edge = edge.nextController;
      if (edge.prevBody) edge.prevBody.nextBody = edge.nextBody;
      if (edge.nextBody) edge.nextBody.prevBody = edge.prevBody;
      če (edge.nextController)
        edge.nextController.prevController = edge.prevController;
      če (edge.prevController)
        edge.prevController.nextController = edge.nextController;
      if (this.m_bodyList == rob) this.m_bodyList = edge.nextBody;
      če (body.m_controllerList == rob)
        body.m_controllerList = edge.nextController;
      body.m_controllerCount--;
      this.m_bodyCount--;
    };
    b2Controller.prototype.Clear = funkcija () {
      medtem ko (this.m_bodyList) this.RemoveBody(this.m_bodyList.body);
    };
    b2Controller.prototype.GetNext = funkcija () {
      vrni this.m_next;
    };
    b2Controller.prototype.GetWorld = funkcija () {
      vrni this.m_world;
    };
    b2Controller.prototype.GetBodyList = funkcija () {
      vrni this.m_bodyList;
    };
    b2ControllerEdge.b2ControllerEdge = funkcija () {};
    Box2D.inherit(b2GravityController, Box2D.Dynamics.Controllers.b2Controller);
    b2GravityController.prototype.__super =
      Box2D.Dynamics.Controllers.b2Controller.prototype;
    b2GravityController.b2GravityController = funkcija () {
      Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply(this, arguments);
      to.G = 1;
      this.invSqr = res;
    };
    b2GravityController.prototype.Step = funkcija (korak) {
      var i = nič;
      var body1 = null;
      var p1 = nič;
      var. masa1 = 0;
      var j = nič;
      var body2 = null;
      var p2 = nič;
      var dx = 0;
      var dy = 0;
      var r2 = 0;
      var f = nič;
      if (this.invSqr) {
        for (i = this.m_bodyList; i; i = i.nextBody) {
          telo1 = i.telo;
          p1 = body1.GetWorldCenter();
          masa1 = telo1.GetMass();
          for (j = this.m_bodyList; j != i; j = j.nextBody) {
            telo2 = j.telo;
            p2 = body2.GetWorldCenter();
            dx = p2.x - p1.x;
            dy = p2.y - p1.y;
            r2 = dx * dx + dy * dy;
            if (r2 < Number.MIN_VALUE) nadaljuje;
            f = novo b2Vec2(dx, dy);
            f.Množenje((this.G / r2 / Math.sqrt(r2)) * masa1 * telo2.GetMass());
            if (body1.IsAwake()) body1.ApplyForce(f, p1);
            f.Pomnoži(-1);
            if (body2.IsAwake()) body2.ApplyForce(f, p2);
          }
        }
      } drugače {
        for (i = this.m_bodyList; i; i = i.nextBody) {
          telo1 = i.telo;
          p1 = body1.GetWorldCenter();
          masa1 = telo1.GetMass();
          for (j = this.m_bodyList; j != i; j = j.nextBody) {
            telo2 = j.telo;
            p2 = body2.GetWorldCenter();
            dx = p2.x - p1.x;
            dy = p2.y - p1.y;
            r2 = dx * dx + dy * dy;
            if (r2 < Number.MIN_VALUE) nadaljuje;
            f = novo b2Vec2(dx, dy);
            f.Množi ((this.G / r2) * masa1 * telo2.GetMass());
            if (body1.IsAwake()) body1.ApplyForce(f, p1);
            f.Pomnoži(-1);
            if (body2.IsAwake()) body2.ApplyForce(f, p2);
          }
        }
      }
    };
    Box2D.inherit(
      b2TensorDampingController,
      Box2D.Dynamics.Controllers.b2Controller
    );
    b2TensorDampingController.prototype.__super =
      Box2D.Dynamics.Controllers.b2Controller.prototype;
    b2TensorDampingController.b2TensorDampingController = funkcija () {
      Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply(this, arguments);
      this.T = novo b2Mat22();
      this.maxTimestep = 0;
    };
    b2TensorDampingController.prototype.SetAxisAligned = funkcija (
      xDamping,
      yDumping
    ) {
      if (xDamping === nedefinirano) xDamping = 0;
      if (yDamping === nedefinirano) yDamping = 0;
      this.T.col1.x = -xDamping;
      this.T.col1.y = 0;
      this.T.col2.x = 0;
      this.T.col2.y = -yDumping;
      if (xDamping > 0 || yDamping > 0) {
        this.maxTimestep = 1 / Math.max(xDamping, yDamping);
      } drugače {
        this.maxTimestep = 0;
      }
    };
    b2TensorDampingController.prototype.Step = funkcija (korak) {
      var časovni korak = korak.dt;
      if (timestep <= Number.MIN_VALUE) return;
      če (časovni korak > this.maxTimestep && this.maxTimestep > 0)
        časovni korak = this.maxTimestep;
      for (var i = this.m_bodyList; i; i = i.nextBody) {
        var body = i.body;
        if (!body.IsAwake()) {
          nadaljevati;
        }
        var damping = body.GetWorldVector(
          b2Math.MulMV(this.T, body.GetLocalVector(body.GetLinearVelocity()))
        );
        body.SetLinearVelocity(
          novo b2Vec2(
            body.GetLinearVelocity().x + dušenje.x * časovni korak,
            body.GetLinearVelocity().y + dušenje.y * časovni korak
          )
        );
      }
    };
  })();
  (funkcija () {
    var b2Color = Box2D.Common.b2Color,
      b2notranji = Box2D.Common.b2notranji,
      b2Settings = Box2D.Common.b2Settings,
      b2Mat22 = Box2D.Common.Math.b2Mat22,
      b2Mat33 = Box2D.Common.Math.b2Mat33,
      b2Math = Box2D.Common.Math.b2Math,
      b2Sweep = Box2D.Common.Math.b2Sweep,
      b2Transform = Box2D.Common.Math.b2Transform,
      b2Vec2 = Box2D.Common.Math.b2Vec2,
      b2Vec3 = Box2D.Common.Math.b2Vec3,
      b2DistanceJoint = Box2D.Dynamics.Joints.b2DistanceJoint,
      b2DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef,
      b2FrictionJoint = Box2D.Dynamics.Joints.b2FrictionJoint,
      b2FrictionJointDef = Box2D.Dynamics.Joints.b2FrictionJointDef,
      b2GearJoint = Box2D.Dynamics.Joints.b2GearJoint,
      b2GearJointDef = Box2D.Dynamics.Joints.b2GearJointDef,
      b2Jacobian = Box2D.Dynamics.Joints.b2Jacobian,
      b2Joint = Box2D.Dynamics.Joints.b2Joint,
      b2JointDef = Box2D.Dynamics.Joints.b2JointDef,
      b2JointEdge = Box2D.Dynamics.Joints.b2JointEdge,
      b2LineJoint = Box2D.Dynamics.Joints.b2LineJoint,
      b2LineJointDef = Box2D.Dynamics.Joints.b2LineJointDef,
      b2MouseJoint = Box2D.Dynamics.Joints.b2MouseJoint,
      b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef,
      b2PrismaticJoint = Box2D.Dynamics.Joints.b2PrismaticJoint,
      b2PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef,
      b2PulleyJoint = Box2D.Dynamics.Joints.b2PulleyJoint,
      b2PulleyJointDef = Box2D.Dynamics.Joints.b2PulleyJointDef,
      b2RevoluteJoint = Box2D.Dynamics.Joints.b2RevoluteJoint,
      b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef,
      b2WeldJoint = Box2D.Dynamics.Joints.b2WeldJoint,
      b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef,
      b2Body = Box2D.Dynamics.b2Body,
      b2BodyDef = Box2D.Dynamics.b2BodyDef,
      b2ContactFilter = Box2D.Dynamics.b2ContactFilter,
      b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse,
      b2ContactListener = Box2D.Dynamics.b2ContactListener,
      b2ContactManager = Box2D.Dynamics.b2ContactManager,
      b2DebugDraw = Box2D.Dynamics.b2DebugDraw,
      b2DestructionListener = Box2D.Dynamics.b2DestructionListener,
      b2FilterData = Box2D.Dynamics.b2FilterData,
      b2Fixture = Box2D.Dynamics.b2Fixture,
      b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
      b2Island = Box2D.Dynamics.b2Island,
      b2TimeStep = Box2D.Dynamics.b2TimeStep,
      b2Svet = Box2D.Dynamics.b2Svet;

    Box2D.inherit(b2DistanceJoint, Box2D.Dynamics.Joints.b2Joint);
    b2DistanceJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2DistanceJoint.b2DistanceJoint = funkcija () {
      Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
      this.m_localAnchor1 = novo b2Vec2();
      this.m_localAnchor2 = novo b2Vec2();
      this.m_u = novo b2Vec2();
    };
    b2DistanceJoint.prototype.GetAnchorA = funkcija () {
      vrni this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
    };
    b2DistanceJoint.prototype.GetAnchorB = funkcija () {
      vrni this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
    };
    b2DistanceJoint.prototype.GetReactionForce = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrni novo b2Vec2(
        inv_dt * this.m_impulse * this.m_u.x,
        inv_dt * ta.m_impulz * ta.m_u.y
      );
    };
    b2DistanceJoint.prototype.GetReactionTorque = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrnitev 0,0;
    };
    b2DistanceJoint.prototype.GetLength = funkcija () {
      vrni this.m_length;
    };
    b2DistanceJoint.prototype.SetLength = funkcija (dolžina) {
      if (dolžina === nedefinirano) length = 0;
      this.m_length = dolžina;
    };
    b2DistanceJoint.prototype.GetFrequency = funkcija () {
      vrni this.m_frequencyHz;
    };
    b2DistanceJoint.prototype.SetFrequency = funkcija (hz) {
      if (hz === nedefinirano) hz = 0;
      this.m_frekvencaHz = hz;
    };
    b2DistanceJoint.prototype.GetDampingRatio = funkcija () {
      vrni this.m_dampingRatio;
    };
    b2DistanceJoint.prototype.SetDampingRatio = funkcija (razmerje) {
      če (razmerje === nedefinirano) razmerje = 0;
      this.m_dampingRatio = razmerje;
    };
    b2DistanceJoint.prototype.b2DistanceJoint = funkcija (def) {
      this.__super.b2Joint.call(this, def);
      var tMat;
      var tX = 0;
      var tY = 0;
      this.m_localAnchor1.SetV(def.localAnchorA);
      this.m_localAnchor2.SetV(def.localAnchorB);
      this.m_length = def.length;
      this.m_frequencyHz = def.frequencyHz;
      this.m_dampingRatio = def.dampingRatio;
      this.m_impulz = 0,0;
      this.m_gamma = 0,0;
      this.m_bias = 0,0;
    };
    b2DistanceJoint.prototype.InitVelocityConstraints = funkcija (korak) {
      var tMat;
      var tX = 0;
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
      var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
      var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      this.m_u.x = bB.m_sweep.cx + r2X - bA.m_sweep.cx - r1X;
      this.m_u.y = bB.m_sweep.cy + r2Y - bA.m_sweep.cy - r1Y;
      var length = Math.sqrt(this.m_u.x * this.m_u.x + this.m_u.y * this.m_u.y);
      if (dolžina > b2Settings.b2_linearSlop) {
        this.m_u.Množi (1,0 / dolžina);
      } drugače {
        this.m_u.SetZero();
      }
      var cr1u = r1X * this.m_u.y - r1Y * this.m_u.x;
      var cr2u = r2X * this.m_u.y - r2Y * this.m_u.x;
      var invMass =
        bA.m_invMass +
        bA.m_invI * cr1u * cr1u +
        bB.m_invMass +
        bB.m_invI * cr2u * cr2u;
      this.m_mass = invMass != 0,0 ? 1,0 / invMasa: 0,0;
      if (this.m_frequencyHz > 0,0) {
        var C = dolžina - this.m_length;
        var omega = 2.0 * Math.PI * this.m_frequencyHz;
        var d = 2,0 * this.m_mass * this.m_dampingRatio * omega;
        var k = this.m_mass * omega * omega;
        this.m_gamma = step.dt * (d + step.dt * k);
        this.m_gamma = this.m_gamma != 0.0 ? 1 / this.m_gamma : 0,0;
        this.m_bias = C * step.dt * k * this.m_gamma;
        this.m_mass = invMass + this.m_gamma;
        this.m_mass = this.m_mass != 0,0 ? 1,0 / ta.m_masa: 0,0;
      }
      if (step.warmStarting) {
        this.m_impulse *= step.dtRatio;
        var PX = this.m_impulse * this.m_u.x;
        var PY = this.m_impulse * this.m_u.y;
        bA.m_linearVelocity.x -= bA.m_invMass * PX;
        bA.m_linearnaHitrost.y -= bA.m_invMasa * PY;
        bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX);
        bB.m_linearVelocity.x += bB.m_invMass * PX;
        bB.m_linearVelocity.y += bB.m_invMass * PY;
        bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX);
      } drugače {
        this.m_impulz = 0,0;
      }
    };
    b2DistanceJoint.prototype.SolveVelocityConstraints = funkcija (korak) {
      var tMat;
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
      var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      var tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
      var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      var v1X = bA.m_linearVelocity.x + -bA.m_angularVelocity * r1Y;
      var v1Y = bA.m_linearVelocity.y + bA.m_angularVelocity * r1X;
      var v2X = bB.m_linearVelocity.x + -bB.m_angularVelocity * r2Y;
      var v2Y = bB.m_linearVelocity.y + bB.m_angularVelocity * r2X;
      var Cdot = this.m_u.x * (v2X - v1X) + this.m_u.y * (v2Y - v1Y);
      var impulz =
        -this.m_mass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse);
      this.m_impulse += impulz;
      var PX = impulz * this.m_u.x;
      var PY = impulz * this.m_u.y;
      bA.m_linearVelocity.x -= bA.m_invMass * PX;
      bA.m_linearnaHitrost.y -= bA.m_invMasa * PY;
      bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX);
      bB.m_linearVelocity.x += bB.m_invMass * PX;
      bB.m_linearVelocity.y += bB.m_invMass * PY;
      bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX);
    };
    b2DistanceJoint.prototype.SolvePositionConstraints = funkcija (baumgarte) {
      if (baumgarte === nedefinirano) baumgarte = 0;
      var tMat;
      if (this.m_frequencyHz > 0,0) {
        vrni resnico;
      }
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
      var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      var tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
      var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      var dX = bB.m_sweep.cx + r2X - bA.m_sweep.cx - r1X;
      var dY = bB.m_sweep.cy + r2Y - bA.m_sweep.cy - r1Y;
      var length = Math.sqrt(dX * dX + dY * dY);
      dX /= dolžina;
      dY /= dolžina;
      var C = dolžina - this.m_length;
      C = b2Math.Clamp(
        C,
        -b2Settings.b2_maxLinearCorrection,
        b2Settings.b2_maxLinearCorrection
      );
      var impulz = -this.m_mass * C;
      this.m_u.Set(dX, dY);
      var PX = impulz * this.m_u.x;
      var PY = impulz * this.m_u.y;
      bA.m_sweep.cx -= bA.m_invMass * PX;
      bA.m_sweep.cy -= bA.m_invMass * PY;
      bA.m_sweep.a -= bA.m_invI * (r1X * PY - r1Y * PX);
      bB.m_sweep.cx += bB.m_invMass * PX;
      bB.m_sweep.cy += bB.m_invMass * PY;
      bB.m_sweep.a += bB.m_invI * (r2X * PY - r2Y * PX);
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
      return b2Math.Abs(C) < b2Settings.b2_linearSlop;
    };
    Box2D.inherit(b2DistanceJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2DistanceJointDef.prototype.__super =
      Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2DistanceJointDef.b2DistanceJointDef = funkcija () {
      Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(to, argumenti);
      this.localAnchorA = novo b2Vec2();
      this.localAnchorB = novo b2Vec2();
    };
    b2DistanceJointDef.prototype.b2DistanceJointDef = funkcija () {
      this.__super.b2JointDef.call(this);
      this.type = b2Joint.e_distanceJoint;
      this.length = 1,0;
      this.frequencyHz = 0,0;
      this.dampingRatio = 0,0;
    };
    b2DistanceJointDef.prototype.Initialize = funkcija (
      bA,
      bB,
      sidroA,
      sidroB
    ) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchorA));
      this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchorB));
      var dX = sidroB.x - sidroA.x;
      var dY = sidroB.y - sidroA.y;
      this.length = Math.sqrt(dX * dX + dY * dY);
      this.frequencyHz = 0,0;
      this.dampingRatio = 0,0;
    };
    Box2D.inherit(b2FrictionJoint, Box2D.Dynamics.Joints.b2Joint);
    b2FrictionJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2FrictionJoint.b2FrictionJoint = funkcija () {
      Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
      this.m_localAnchorA = novo b2Vec2();
      this.m_localAnchorB = novo b2Vec2();
      this.m_linearMass = novo b2Mat22();
      this.m_linearImpulse = novo b2Vec2();
    };
    b2FrictionJoint.prototype.GetAnchorA = funkcija () {
      vrni this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
    };
    b2FrictionJoint.prototype.GetAnchorB = funkcija () {
      vrni this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
    };
    b2FrictionJoint.prototype.GetReactionForce = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrni novo b2Vec2(
        inv_dt * this.m_linearImpulse.x,
        inv_dt * this.m_linearImpulse.y
      );
    };
    b2FrictionJoint.prototype.GetReactionTorque = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      return inv_dt * this.m_angularImpulse;
    };
    b2FrictionJoint.prototype.SetMaxForce = funkcija (sila) {
      if (sila === nedefinirano) sila = 0;
      this.m_maxForce = sila;
    };
    b2FrictionJoint.prototype.GetMaxForce = funkcija () {
      vrni this.m_maxForce;
    };
    b2FrictionJoint.prototype.SetMaxTorque = funkcija (navor) {
      če (navor === nedefiniran) navor = 0;
      this.m_maxTorque = navor;
    };
    b2FrictionJoint.prototype.GetMaxTorque = funkcija () {
      vrni this.m_maxTorque;
    };
    b2FrictionJoint.prototype.b2FrictionJoint = funkcija (def) {
      this.__super.b2Joint.call(this, def);
      this.m_localAnchorA.SetV(def.localAnchorA);
      this.m_localAnchorB.SetV(def.localAnchorB);
      this.m_linearMass.SetZero();
      this.m_angularMass = 0,0;
      this.m_linearImpulse.SetZero();
      this.m_angularImpulse = 0,0;
      this.m_maxForce = def.maxForce;
      this.m_maxTorque = def.maxTorque;
    };
    b2FrictionJoint.prototype.InitVelocityConstraints = funkcija (korak) {
      var tMat;
      var tX = 0;
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      tMat = bA.m_xf.R;
      var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
      var rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
      tX = tMat.col1.x * rAX + tMat.col2.x * rAY;
      rAY = tMat.col1.y * rAX + tMat.col2.y * rAY;
      rAX = tX;
      tMat = bB.m_xf.R;
      var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
      var rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * rBX + tMat.col2.x * rBY;
      rBY = tMat.col1.y * rBX + tMat.col2.y * rBY;
      rBX = tX;
      var mA = bA.m_invMass;
      var mB = bB.m_invMass;
      var iA = bA.m_invI;
      var iB = bB.m_invI;
      var K = novo b2Mat22();
      K.col1.x = mA + mB;
      K.col2.x = 0,0;
      K.col1.y = 0,0;
      K.col2.y = mA + mB;
      K.col1.x += iA * rAY * rAY;
      K.col2.x += -iA * rAX * rAY;
      K.col1.y += -iA * rAX * rAY;
      K.col2.y += iA * rAX * rAX;
      K.col1.x += iB * rBY * rBY;
      K.col2.x += -iB * rBX * rBY;
      K.col1.y += -iB * rBX * rBY;
      K.col2.y += iB * rBX * rBX;
      K.GetInverse(this.m_linearMass);
      this.m_angularMass = iA + iB;
      if (this.m_angularMass > 0,0) {
        this.m_angularMass = 1,0 / this.m_angularMass;
      }
      if (step.warmStarting) {
        this.m_linearImpulse.x *= step.dtRatio;
        this.m_linearImpulse.y *= step.dtRatio;
        this.m_angularImpulse *= step.dtRatio;
        var P = this.m_linearImpulse;
        bA.m_linearVelocity.x -= mA * Px;
        bA.m_linearnaHitrost.y -= mA * Py;
        bA.m_angularVelocity -=
          iA * (rAX * Py - rAY * Px + this.m_angularImpulse);
        bB.m_linearVelocity.x += mB * Px;
        bB.m_linearVelocity.y += mB * Py;
        bB.m_angularVelocity +=
          iB * (rBX * Py - rBY * Px + this.m_angularImpulse);
      } drugače {
        this.m_linearImpulse.SetZero();
        this.m_angularImpulse = 0,0;
      }
    };
    b2FrictionJoint.prototype.SolveVelocityConstraints = funkcija (korak) {
      var tMat;
      var tX = 0;
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var vA = bA.m_linearna hitrost;
      var wA = bA.m_angularVelocity;
      var vB = bB.m_linearnaHitrost;
      var wB = bB.m_angularVelocity;
      var mA = bA.m_invMass;
      var mB = bB.m_invMass;
      var iA = bA.m_invI;
      var iB = bB.m_invI;
      tMat = bA.m_xf.R;
      var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
      var rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
      tX = tMat.col1.x * rAX + tMat.col2.x * rAY;
      rAY = tMat.col1.y * rAX + tMat.col2.y * rAY;
      rAX = tX;
      tMat = bB.m_xf.R;
      var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
      var rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * rBX + tMat.col2.x * rBY;
      rBY = tMat.col1.y * rBX + tMat.col2.y * rBY;
      rBX = tX;
      var maxImpulse = 0;
      {
        var Cdot = wB - wA;
        var impulz = -this.m_angularMass * Cdot;
        var oldImpulse = this.m_angularImpulse;
        maxImpulse = step.dt * this.m_maxTorque;
        this.m_angularImpulse = b2Math.Clamp(
          this.m_angularImpulse + impulz,
          -največji impulz,
          maxImpulse
        );
        impulz = this.m_angularImpulse - oldImpulse;
        wA -= iA * impulz;
        wB += iB * impulz;
      }
      {
        var CdotX = vB.x - wB * rBY - vA.x + wA * rAY;
        var CdotY = vB.y + wB * rBX - vA.y - wA * rAX;
        var impulzV = b2Math.MulMV(
          this.m_linearMass,
          novo b2Vec2(-CdotX, -CdotY)
        );
        var oldImpulseV = this.m_linearImpulse.Copy();
        this.m_linearImpulse.Add(impulseV);
        maxImpulse = step.dt * this.m_maxForce;
        if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
          this.m_linearImpulse.Normalize();
          this.m_linearImpulse.Multiply(maxImpulse);
        }
        impulzV = b2Math.SubtractVV(this.m_linearniImpulz, stariImpulzV);
        vA.x -= mA * impulzV.x;
        vA.y -= mA * impulzV.y;
        wA -= iA * (rAX * impulzV.y - rAY * impulzV.x);
        vB.x += mB * impulzV.x;
        vB.y += mB * impulzV.y;
        wB += iB * (rBX * impulzV.y - rBY * impulzV.x);
      }
      bA.m_angularVelocity = wA;
      bB.m_angularVelocity = wB;
    };
    b2FrictionJoint.prototype.SolvePositionConstraints = funkcija (baumgarte) {
      if (baumgarte === nedefinirano) baumgarte = 0;
      vrni resnico;
    };
    Box2D.inherit(b2FrictionJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2FrictionJointDef.prototype.__super =
      Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2FrictionJointDef.b2FrictionJointDef = funkcija () {
      Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(to, argumenti);
      this.localAnchorA = novo b2Vec2();
      this.localAnchorB = novo b2Vec2();
    };
    b2FrictionJointDef.prototype.b2FrictionJointDef = funkcija () {
      this.__super.b2JointDef.call(this);
      this.type = b2Joint.e_frictionJoint;
      this.maxForce = 0,0;
      this.maxTorque = 0,0;
    };
    b2FrictionJointDef.prototype.Initialize = funkcija (bA, bB, sidro) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA.SetV(this.bodyA.GetLocalPoint(sidro));
      this.localAnchorB.SetV(this.bodyB.GetLocalPoint(sidro));
    };
    Box2D.inherit(b2GearJoint, Box2D.Dynamics.Joints.b2Joint);
    b2GearJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2GearJoint.b2GearJoint = funkcija () {
      Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
      this.m_groundAnchor1 = novo b2Vec2();
      this.m_groundAnchor2 = novo b2Vec2();
      this.m_localAnchor1 = novo b2Vec2();
      this.m_localAnchor2 = novo b2Vec2();
      this.m_J = novo b2Jacobian();
    };
    b2GearJoint.prototype.GetAnchorA = funkcija () {
      vrni this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
    };
    b2GearJoint.prototype.GetAnchorB = funkcija () {
      vrni this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
    };
    b2GearJoint.prototype.GetReactionForce = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrni novo b2Vec2(
        inv_dt * this.m_impulse * this.m_J.linearB.x,
        inv_dt * this.m_impulse * this.m_J.linearB.y
      );
    };
    b2GearJoint.prototype.GetReactionTorque = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      var tMat = this.m_bodyB.m_xf.R;
      var rX = this.m_localAnchor1.x - this.m_bodyB.m_sweep.localCenter.x;
      var rY = this.m_localAnchor1.y - this.m_bodyB.m_sweep.localCenter.y;
      var tX = tMat.col1.x * rX + tMat.col2.x * rY;
      rY = tMat.col1.y * rX + tMat.col2.y * rY;
      rX = tX;
      var PX = this.m_impulse * this.m_J.linearB.x;
      var PY = this.m_impulse * this.m_J.linearB.y;
      vrni inv_dt * (this.m_impulse * this.m_J.angularB - rX ​​* PY + rY * PX);
    };
    b2GearJoint.prototype.GetRatio = funkcija () {
      vrni this.m_ratio;
    };
    b2GearJoint.prototype.SetRatio = funkcija (razmerje) {
      če (razmerje === nedefinirano) razmerje = 0;
      this.m_ratio = razmerje;
    };
    b2GearJoint.prototype.b2GearJoint = funkcija (def) {
      this.__super.b2Joint.call(this, def);
      var type1 = parseInt(def.joint1.m_type);
      var type2 = parseInt(def.joint2.m_type);
      this.m_revolute1 = nič;
      this.m_prismatic1 = nič;
      this.m_revolute2 = nič;
      this.m_prismatic2 = nič;
      spremenljiva koordinata1 = 0;
      spremenljiva koordinata2 = 0;
      this.m_ground1 = def.joint1.GetBodyA();
      this.m_bodyA = def.joint1.GetBodyB();
      if (type1 == b2Joint.e_revoluteJoint) {
        this.m_revolute1 =
          def.joint1 primerek b2RevoluteJoint? def.joint1: nič;
        this.m_groundAnchor1.SetV(this.m_revolute1.m_localAnchor1);
        this.m_localAnchor1.SetV(this.m_revolute1.m_localAnchor2);
        koordinata1 = this.m_revolute1.GetJointAngle();
      } drugače {
        this.m_prismatic1 =
          def.joint1 primerek b2PrismaticJoint? def.joint1: nič;
        this.m_groundAnchor1.SetV(this.m_prismatic1.m_localAnchor1);
        this.m_localAnchor1.SetV(this.m_prismatic1.m_localAnchor2);
        koordinata1 = this.m_prismatic1.GetJointTranslation();
      }
      this.m_ground2 = def.joint2.GetBodyA();
      this.m_bodyB = def.joint2.GetBodyB();
      if (type2 == b2Joint.e_revoluteJoint) {
        this.m_revolute2 =
          def.joint2 instanceof b2RevoluteJoint? def.joint2 : nič;
        this.m_groundAnchor2.SetV(this.m_revolute2.m_localAnchor1);
        this.m_localAnchor2.SetV(this.m_revolute2.m_localAnchor2);
        koordinata2 = this.m_revolute2.GetJointAngle();
      } drugače {
        this.m_prismatic2 =
          def.joint2 instanceof b2PrismaticJoint? def.joint2 : nič;
        this.m_groundAnchor2.SetV(this.m_prismatic2.m_localAnchor1);
        this.m_localAnchor2.SetV(this.m_prismatic2.m_localAnchor2);
        koordinata2 = this.m_prismatic2.GetJointTranslation();
      }
      this.m_ratio = def.ratio;
      this.m_constant = koordinata1 + this.m_ratio * koordinata2;
      this.m_impulz = 0,0;
    };
    b2GearJoint.prototype.InitVelocityConstraints = funkcija (korak) {
      var g1 = this.m_ground1;
      var g2 = this.m_ground2;
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var ugX = 0;
      var ugY = 0;
      var rX = 0;
      var rY = 0;
      var tMat;
      var tVec;
      var crug = 0;
      var tX = 0;
      var K = 0,0;
      this.m_J.SetZero();
      if (this.m_revolute1) {
        this.m_J.angularA = -1,0;
        K += bA.m_invI;
      } drugače {
        tMat = g1.m_xf.R;
        tVec = this.m_prismatic1.m_localXAxis1;
        ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tMat = bA.m_xf.R;
        rX = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
        rY = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
        tX = tMat.col1.x * rX + tMat.col2.x * rY;
        rY = tMat.col1.y * rX + tMat.col2.y * rY;
        rX = tX;
        križ = rX * ugY - rY * ugX;
        this.m_J.linearA.Set(-ugX, -ugY);
        this.m_J.angularA = -crug;
        K += bA.m_invMasa + bA.m_invI * križ * križ;
      }
      if (this.m_revolute2) {
        this.m_J.angularB = -this.m_ratio;
        K += this.m_ratio * this.m_ratio * bB.m_invI;
      } drugače {
        tMat = g2.m_xf.R;
        tVec = this.m_prismatic2.m_localXAxis1;
        ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
        ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
        tMat = bB.m_xf.R;
        rX = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
        rY = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
        tX = tMat.col1.x * rX + tMat.col2.x * rY;
        rY = tMat.col1.y * rX + tMat.col2.y * rY;
        rX = tX;
        križ = rX * ugY - rY * ugX;
        this.m_J.linearB.Set(-this.m_ratio * ugX, -this.m_ratio * ugY);
        this.m_J.angularB = -this.m_ratio * crug;
        K +=
          this.m_ratio * this.m_ratio * (bB.m_invMass + bB.m_invI * crug * crug);
      }
      this.m_mass = K > 0,0? 1,0 / K : 0,0;
      if (step.warmStarting) {
        bA.m_linearVelocity.x +=
          bA.m_invMass * this.m_impulse * this.m_J.linearA.x;
        bA.m_linearnaHitrost.y +=
          bA.m_invMass * this.m_impulse * this.m_J.linearA.y;
        bA.m_angularVelocity += bA.m_invI * this.m_impulse * this.m_J.angularA;
        bB.m_linearVelocity.x +=
          bB.m_invMass * this.m_impulse * this.m_J.linearB.x;
        bB.m_linearVelocity.y +=
          bB.m_invMass * this.m_impulse * this.m_J.linearB.y;
        bB.m_angularVelocity += bB.m_invI * this.m_impulse * this.m_J.angularB;
      } drugače {
        this.m_impulz = 0,0;
      }
    };
    b2GearJoint.prototype.SolveVelocityConstraints = funkcija (korak) {
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var Cdot = this.m_J.Compute(
        bA.m_linearna hitrost,
        bA.m_angularVelocity,
        bB.m_linearna hitrost,
        bB.m_angularVelocity
      );
      var impulz = -this.m_mass * Cdot;
      this.m_impulse += impulz;
      bA.m_linearVelocity.x += bA.m_invMass * impulz * this.m_J.linearA.x;
      bA.m_linearVelocity.y += bA.m_invMass * impulz * this.m_J.linearA.y;
      bA.m_angularVelocity += bA.m_invI * impulz * this.m_J.angularA;
      bB.m_linearVelocity.x += bB.m_invMass * impulz * this.m_J.linearB.x;
      bB.m_linearVelocity.y += bB.m_invMass * impulz * this.m_J.linearB.y;
      bB.m_angularVelocity += bB.m_invI * impulz * this.m_J.angularB;
    };
    b2GearJoint.prototype.SolvePositionConstraints = funkcija (baumgarte) {
      if (baumgarte === nedefinirano) baumgarte = 0;
      var linearError = 0,0;
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      spremenljiva koordinata1 = 0;
      spremenljiva koordinata2 = 0;
      if (this.m_revolute1) {
        koordinata1 = this.m_revolute1.GetJointAngle();
      } drugače {
        koordinata1 = this.m_prismatic1.GetJointTranslation();
      }
      if (this.m_revolute2) {
        koordinata2 = this.m_revolute2.GetJointAngle();
      } drugače {
        koordinata2 = this.m_prismatic2.GetJointTranslation();
      }
      var C = this.m_constant - (koordinata1 + this.m_razmerje * koordinata2);
      var impulz = -this.m_mass * C;
      bA.m_sweep.cx += bA.m_invMass * impulz * this.m_J.linearA.x;
      bA.m_sweep.cy += bA.m_invMass * impulz * this.m_J.linearA.y;
      bA.m_sweep.a += bA.m_invI * impulz * this.m_J.angularA;
      bB.m_sweep.cx += bB.m_invMass * impulz * this.m_J.linearB.x;
      bB.m_sweep.cy += bB.m_invMass * impulz * this.m_J.linearB.y;
      bB.m_sweep.a += bB.m_invI * impulz * this.m_J.angularB;
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
      return linearError < b2Settings.b2_linearSlop;
    };
    Box2D.inherit(b2GearJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2GearJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2GearJointDef.b2GearJointDef = funkcija () {
      Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(to, argumenti);
    };
    b2GearJointDef.prototype.b2GearJointDef = funkcija () {
      this.__super.b2JointDef.call(this);
      this.type = b2Joint.e_gearJoint;
      this.joint1 = nič;
      this.joint2 = nič;
      this.ratio = 1,0;
    };
    b2Jacobian.b2Jacobian = funkcija () {
      this.linearA = novo b2Vec2();
      this.linearB = novo b2Vec2();
    };
    b2Jacobian.prototype.SetZero = funkcija () {
      this.linearA.SetZero();
      this.angularA = 0,0;
      this.linearB.SetZero();
      this.angularB = 0,0;
    };
    b2Jacobian.prototype.Set = funkcija (x1, a1, x2, a2) {
      if (a1 === nedefinirano) a1 = 0;
      if (a2 === nedefinirano) a2 = 0;
      this.linearA.SetV(x1);
      this.angularA = a1;
      this.linearB.SetV(x2);
      this.angularB = a2;
    };
    b2Jacobian.prototype.Compute = funkcija (x1, a1, x2, a2) {
      if (a1 === nedefinirano) a1 = 0;
      if (a2 === nedefinirano) a2 = 0;
      vrnitev (
        this.linearA.x * x1.x +
        this.linearA.y * x1.y +
        to.kotnoA * a1 +
        (this.linearB.x * x2.x + this.linearB.y * x2.y) +
        ta.kotniB * a2
      );
    };
    b2Joint.b2Joint = funkcija () {
      this.m_edgeA = novo b2JointEdge();
      this.m_edgeB = novo b2JointEdge();
      this.m_localCenterA = novo b2Vec2();
      this.m_localCenterB = novo b2Vec2();
    };
    b2Joint.prototype.GetType = funkcija () {
      vrni this.m_type;
    };
    b2Joint.prototype.GetAnchorA = funkcija () {
      vrni nič;
    };
    b2Joint.prototype.GetAnchorB = funkcija () {
      vrni nič;
    };
    b2Joint.prototype.GetReactionForce = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrni nič;
    };
    b2Joint.prototype.GetReactionTorque = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrnitev 0,0;
    };
    b2Joint.prototype.GetBodyA = funkcija () {
      vrni this.m_bodyA;
    };
    b2Joint.prototype.GetBodyB = funkcija () {
      vrni this.m_bodyB;
    };
    b2Joint.prototype.GetNext = funkcija () {
      vrni this.m_next;
    };
    b2Joint.prototype.GetUserData = funkcija () {
      vrni this.m_userData;
    };
    b2Joint.prototype.SetUserData = funkcija (podatki) {
      this.m_userData = podatki;
    };
    b2Joint.prototype.IsActive = funkcija () {
      vrni this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
    };
    b2Joint.Create = funkcija (def, alokator) {
      var joint = null;
      stikalo (def.type) {
        case b2Joint.e_distanceJoint:
          {
            joint = new b2DistanceJoint(
              def instanceof b2DistanceJointDef? def: nič
            );
          }
          odmor;
        case b2Joint.e_mouseJoint:
          {
            joint = new b2MouseJoint(def instanceof b2MouseJointDef? def: null);
          }
          odmor;
        primer b2Joint.e_prismaticJoint:
          {
            joint = new b2PrismaticJoint(
              def instanceof b2PrismaticJointDef? def: nič
            );
          }
          odmor;
        primer b2Joint.e_revoluteJoint:
          {
            joint = new b2RevoluteJoint(
              def instanceof b2RevoluteJointDef? def: nič
            );
          }
          odmor;
        primer b2Joint.e_pulleyJoint:
          {
            joint = new b2PulleyJoint(
              def instanceof b2PulleyJointDef? def: nič
            );
          }
          odmor;
        primer b2Joint.e_gearJoint:
          {
            joint = new b2GearJoint(def instanceof b2GearJointDef ? def: null);
          }
          odmor;
        primer b2Joint.e_lineJoint:
          {
            joint = new b2LineJoint(def instanceof b2LineJointDef ? def: null);
          }
          odmor;
        primer b2Joint.e_weldJoint:
          {
            joint = new b2WeldJoint(def instanceof b2WeldJointDef? def: null);
          }
          odmor;
        primer b2Joint.e_frictionJoint:
          {
            joint = new b2FrictionJoint(
              def instanceof b2FrictionJointDef? def: nič
            );
          }
          odmor;
        privzeto:
          odmor;
      }
      povratni spoj;
    };
    b2Joint.Destroy = funkcija (joint, alokator) {};
    b2Joint.prototype.b2Joint = funkcija (def) {
      b2Settings.b2Assert(def.bodyA != def.bodyB);
      this.m_type = def.type;
      this.m_prev = null;
      this.m_next = null;
      this.m_bodyA = def.bodyA;
      this.m_bodyB = def.bodyB;
      this.m_collideConnected = def.collideConnected;
      this.m_islandFlag = false;
      this.m_userData = def.userData;
    };
    b2Joint.prototype.InitVelocityConstraints = funkcija (korak) {};
    b2Joint.prototype.SolveVelocityConstraints = funkcija (korak) {};
    b2Joint.prototype.FinalizeVelocityConstraints = funkcija () {};
    b2Joint.prototype.SolvePositionConstraints = funkcija (baumgarte) {
      if (baumgarte === nedefinirano) baumgarte = 0;
      vrni false;
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Dynamics.Joints.b2Joint.e_unknownJoint = 0;
      Box2D.Dynamics.Joints.b2Joint.e_revoluteJoint = 1;
      Box2D.Dynamics.Joints.b2Joint.e_prismaticJoint = 2;
      Box2D.Dynamics.Joints.b2Joint.e_distanceJoint = 3;
      Box2D.Dynamics.Joints.b2Joint.e_pulleyJoint = 4;
      Box2D.Dynamics.Joints.b2Joint.e_mouseJoint = 5;
      Box2D.Dynamics.Joints.b2Joint.e_gearJoint = 6;
      Box2D.Dynamics.Joints.b2Joint.e_lineJoint = 7;
      Box2D.Dynamics.Joints.b2Joint.e_weldJoint = 8;
      Box2D.Dynamics.Joints.b2Joint.e_frictionJoint = 9;
      Box2D.Dynamics.Joints.b2Joint.e_inactiveLimit = 0;
      Box2D.Dynamics.Joints.b2Joint.e_atLowerLimit = 1;
      Box2D.Dynamics.Joints.b2Joint.e_atUpperLimit = 2;
      Box2D.Dynamics.Joints.b2Joint.e_equalLimits = 3;
    });
    b2JointDef.b2JointDef = funkcija () {};
    b2JointDef.prototype.b2JointDef = funkcija () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
    };
    b2JointEdge.b2JointEdge = funkcija () {};
    Box2D.inherit(b2LineJoint, Box2D.Dynamics.Joints.b2Joint);
    b2LineJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2LineJoint.b2LineJoint = funkcija () {
      Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
      this.m_localAnchor1 = novo b2Vec2();
      this.m_localAnchor2 = novo b2Vec2();
      this.m_localXAxis1 = novo b2Vec2();
      this.m_localYAxis1 = novo b2Vec2();
      this.m_axis = new b2Vec2();
      this.m_perp = novo b2Vec2();
      this.m_K = novo b2Mat22();
      this.m_impulse = novo b2Vec2();
    };
    b2LineJoint.prototype.GetAnchorA = funkcija () {
      vrni this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
    };
    b2LineJoint.prototype.GetAnchorB = funkcija () {
      vrni this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
    };
    b2LineJoint.prototype.GetReactionForce = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrni novo b2Vec2(
        inv_dt *
          (this.m_impulse.x * this.m_perp.x +
            (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.x),
        inv_dt *
          (this.m_impulse.x * this.m_perp.y +
            (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.y)
      );
    };
    b2LineJoint.prototype.GetReactionTorque = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      return inv_dt * this.m_impulse.y;
    };
    b2LineJoint.prototype.GetJointTranslation = funkcija () {
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var tMat;
      var p1 = bA.GetWorldPoint(this.m_localAnchor1);
      var p2 = bB.GetWorldPoint(this.m_localAnchor2);
      var dX = p2.x - p1.x;
      var dY = p2.y - p1.y;
      var axis = bA.GetWorldVector(this.m_localXAxis1);
      var prevod = axis.x * dX + axis.y * dY;
      povratni prevod;
    };
    b2LineJoint.prototype.GetJointSpeed ​​= funkcija () {
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var tMat;
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
      var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      var tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
      var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      var p1X = bA.m_sweep.cx + r1X;
      var p1Y = bA.m_sweep.cy + r1Y;
      var p2X = bB.m_sweep.cx + r2X;
      var p2Y = bB.m_sweep.cy + r2Y;
      var dX = p2X - p1X;
      var dY = p2Y - p1Y;
      var axis = bA.GetWorldVector(this.m_localXAxis1);
      var v1 = bA.m_linearna hitrost;
      var v2 = bB.m_linearVelocity;
      var w1 = bA.m_angularVelocity;
      var w2 = bB.m_angularVelocity;
      varna hitrost =
        dX * (-w1 * os.y) +
        dY * (w1 * os.x) +
        (os.x * (v2.x + -w2 * r2Y - v1.x - -w1 * r1Y) +
          os.y * (v2.y + w2 * r2X - v1.y - w1 * r1X));
      povratna hitrost;
    };
    b2LineJoint.prototype.IsLimitEnabled = funkcija () {
      vrni this.m_enableLimit;
    };
    b2LineJoint.prototype.EnableLimit = funkcija (zastavica) {
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_enableLimit = zastavica;
    };
    b2LineJoint.prototype.GetLowerLimit = funkcija () {
      vrni this.m_lowerTranslation;
    };
    b2LineJoint.prototype.GetUpperLimit = funkcija () {
      vrni this.m_upperTranslation;
    };
    b2LineJoint.prototype.SetLimits = funkcija (spodnji, zgornji) {
      če (spodnji === nedefiniran) nižji = 0;
      če (zgornji === nedefinirano) zgornji = 0;
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_lowerTranslation = nižje;
      this.m_upperTranslation = zgornji;
    };
    b2LineJoint.prototype.IsMotorEnabled = funkcija () {
      vrni this.m_enableMotor;
    };
    b2LineJoint.prototype.EnableMotor = funkcija (zastavica) {
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_enableMotor = zastavica;
    };
    b2LineJoint.prototype.SetMotorSpeed ​​= funkcija (hitrost) {
      if (hitrost === nedefinirano) hitrost = 0;
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_motorSpeed ​​= hitrost;
    };
    b2LineJoint.prototype.GetMotorSpeed ​​= funkcija () {
      vrni this.m_motorSpeed;
    };
    b2LineJoint.prototype.SetMaxMotorForce = funkcija (sila) {
      if (sila === nedefinirano) sila = 0;
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_maxMotorForce = sila;
    };
    b2LineJoint.prototype.GetMaxMotorForce = funkcija () {
      vrni this.m_maxMotorForce;
    };
    b2LineJoint.prototype.GetMotorForce = funkcija () {
      vrni this.m_motorImpulse;
    };
    b2LineJoint.prototype.b2LineJoint = funkcija (def) {
      this.__super.b2Joint.call(this, def);
      var tMat;
      var tX = 0;
      var tY = 0;
      this.m_localAnchor1.SetV(def.localAnchorA);
      this.m_localAnchor2.SetV(def.localAnchorB);
      this.m_localXAxis1.SetV(def.localAxisA);
      this.m_localYAxis1.x = -this.m_localXAxis1.y;
      this.m_localYAxis1.y = this.m_localXAxis1.x;
      this.m_impulse.SetZero();
      this.m_motorMass = 0,0;
      this.m_motorImpulse = 0,0;
      this.m_lowerTranslation = def.lowerTranslation;
      this.m_upperTranslation = def.upperTranslation;
      this.m_maxMotorForce = def.maxMotorForce;
      this.m_motorSpeed ​​= def.motorSpeed;
      this.m_enableLimit = def.enableLimit;
      this.m_enableMotor = def.enableMotor;
      this.m_limitState = b2Joint.e_inactiveLimit;
      this.m_axis.SetZero();
      this.m_perp.SetZero();
    };
    b2LineJoint.prototype.InitVelocityConstraints = funkcija (korak) {
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var tMat;
      var tX = 0;
      this.m_localCenterA.SetV(bA.GetLocalCenter());
      this.m_localCenterB.SetV(bB.GetLocalCenter());
      var xf1 = bA.GetTransform();
      var xf2 = bB.GetTransform();
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - this.m_localCenterA.x;
      var r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
      tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - this.m_localCenterB.x;
      var r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      var dX = bB.m_sweep.cx + r2X - bA.m_sweep.cx - r1X;
      var dY = bB.m_sweep.cy + r2Y - bA.m_sweep.cy - r1Y;
      this.m_invMassA = bA.m_invMass;
      this.m_invMassB = bB.m_invMass;
      this.m_invIA = bA.m_invI;
      this.m_invIB = bB.m_invI;
      {
        this.m_axis.SetV(b2Math.MulMV(xf1.R, this.m_localXAxis1));
        this.m_a1 = (dX + r1X) * this.m_os.y - (dY + r1Y) * this.m_os.x;
        this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
        this.m_motorMass =
          this.m_invMassA +
          this.m_invMassB +
          this.m_invIA * this.m_a1 * this.m_a1 +
          this.m_invIB * this.m_a2 * this.m_a2;
        this.m_motorMass =
          this.m_motorMass > Number.MIN_VALUE ? 1,0 / this.m_motorMass : 0,0;
      }
      {
        this.m_perp.SetV(b2Math.MulMV(xf1.R, this.m_localYAxis1));
        this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
        this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
        var m1 = this.m_invMassA;
        var m2 = this.m_invMassB;
        var i1 = this.m_invIA;
        var i2 = this.m_invIB;
        this.m_K.col1.x =
          m1 + m2 + i1 * to.m_s1 * to.m_s1 + i2 * to.m_s2 * to.m_s2;
        this.m_K.col1.y = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
        this.m_K.col2.x = this.m_K.col1.y;
        this.m_K.col2.y =
          m1 + m2 + i1 * to.m_a1 * to.m_a1 + i2 * to.m_a2 * to.m_a2;
      }
      if (this.m_enableLimit) {
        var jointTransition = this.m_axis.x * dX + this.m_axis.y * dY;
        če (
          b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) <
          2.0 * b2Settings.b2_linearSlop
        ) {
          this.m_limitState = b2Joint.e_equalLimits;
        } else if (jointTransition <= this.m_lowerTranslation) {
          if (this.m_limitState != b2Joint.e_atLowerLimit) {
            this.m_limitState = b2Joint.e_atLowerLimit;
            this.m_impulse.y = 0,0;
          }
        } else if (jointTransition >= this.m_upperTranslation) {
          if (this.m_limitState != b2Joint.e_atUpperLimit) {
            this.m_limitState = b2Joint.e_atUpperLimit;
            this.m_impulse.y = 0,0;
          }
        } drugače {
          this.m_limitState = b2Joint.e_inactiveLimit;
          this.m_impulse.y = 0,0;
        }
      } drugače {
        this.m_limitState = b2Joint.e_inactiveLimit;
      }
      if (this.m_enableMotor == false) {
        this.m_motorImpulse = 0,0;
      }
      if (step.warmStarting) {
        this.m_impulse.x *= step.dtRatio;
        this.m_impulse.y *= step.dtRatio;
        this.m_motorImpulse *= step.dtRatio;
        var PX =
          this.m_impulse.x * this.m_perp.x +
          (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.x;
        var PY =
          this.m_impulse.x * this.m_perp.y +
          (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.y;
        var L1 =
          this.m_impulse.x * this.m_s1 +
          (this.m_motorImpulse + this.m_impulse.y) * this.m_a1;
        var L2 =
          this.m_impulse.x * this.m_s2 +
          (this.m_motorImpulse + this.m_impulse.y) * this.m_a2;
        bA.m_linearVelocity.x -= this.m_invMassA * PX;
        bA.m_linearVelocity.y -= this.m_invMassA * PY;
        bA.m_angularVelocity -= this.m_invIA * L1;
        bB.m_linearVelocity.x += this.m_invMassB * PX;
        bB.m_linearVelocity.y += this.m_invMassB * PY;
        bB.m_angularVelocity += this.m_invIB * L2;
      } drugače {
        this.m_impulse.SetZero();
        this.m_motorImpulse = 0,0;
      }
    };
    b2LineJoint.prototype.SolveVelocityConstraints = funkcija (korak) {
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var v1 = bA.m_linearna hitrost;
      var w1 = bA.m_angularVelocity;
      var v2 = bB.m_linearVelocity;
      var w2 = bB.m_angularVelocity;
      var PX = 0;
      var. PY = 0;
      var L1 = 0;
      var L2 = 0;
      if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits) {
        var Cdot =
          this.m_axis.x * (v2.x - v1.x) +
          this.m_axis.y * (v2.y - v1.y) +
          to.m_a2 * w2 -
          this.m_a1 * w1;
        var impulz = this.m_motorMass * (this.m_motorSpeed ​​- Cdot);
        var oldImpulse = this.m_motorImpulse;
        var maxImpulse = step.dt * this.m_maxMotorForce;
        this.m_motorImpulse = b2Math.Clamp(
          this.m_motorImpulse + impulz,
          -največji impulz,
          maxImpulse
        );
        impulz = this.m_motorImpulse - stari impulz;
        PX = impulz * this.m_axis.x;
        PY = impulz * this.m_axis.y;
        L1 = impulz * this.m_a1;
        L2 = impulz * this.m_a2;
        v1.x -= this.m_invMassA * PX;
        v1.y -= this.m_invMassA * PY;
        w1 -= this.m_invIA * L1;
        v2.x += this.m_invMassB * PX;
        v2.y += this.m_invMassB * PY;
        w2 += this.m_invIB * L2;
      }
      var Cdot1 =
        this.m_perp.x * (v2.x - v1.x) +
        this.m_perp.y * (v2.y - v1.y) +
        this.m_s2 * w2 -
        this.m_s1 * w1;
      if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
        var Cdot2 =
          this.m_axis.x * (v2.x - v1.x) +
          this.m_axis.y * (v2.y - v1.y) +
          to.m_a2 * w2 -
          this.m_a1 * w1;
        var f1 = this.m_impulse.Copy();
        var df = this.m_K.Solve(novo b2Vec2(), -Cdot1, -Cdot2);
        this.m_impulse.Add(df);
        if (this.m_limitState == b2Joint.e_atLowerLimit) {
          this.m_impulse.y = b2Math.Max(this.m_impulse.y, 0,0);
        } else if (this.m_limitState == b2Joint.e_atUpperLimit) {
          this.m_impulse.y = b2Math.Min(this.m_impulse.y, 0,0);
        }
        var b = -Cdot1 - (this.m_impulse.y - f1.y) * this.m_K.col2.x;
        var f2r = 0;
        if (this.m_K.col1.x != 0.0) {
          f2r = b / this.m_K.col1.x + f1.x;
        } drugače {
          f2r = f1.x;
        }
        this.m_impulse.x = f2r;
        df.x = this.m_impulse.x - f1.x;
        df.y = ta.m_impulz.y - f1.y;
        PX = df.x * this.m_perp.x + df.y * this.m_axis.x;
        PY = df.x * this.m_perp.y + df.y * this.m_axis.y;
        L1 = df.x * this.m_s1 + df.y * this.m_a1;
        L2 = df.x * this.m_s2 + df.y * this.m_a2;
        v1.x -= this.m_invMassA * PX;
        v1.y -= this.m_invMassA * PY;
        w1 -= this.m_invIA * L1;
        v2.x += this.m_invMassB * PX;
        v2.y += this.m_invMassB * PY;
        w2 += this.m_invIB * L2;
      } drugače {
        var df2 = 0;
        if (this.m_K.col1.x != 0.0) {
          df2 = -Cdot1 / this.m_K.col1.x;
        } drugače {
          df2 = 0,0;
        }
        this.m_impulse.x += df2;
        PX = df2 * this.m_perp.x;
        PY = df2 * this.m_perp.y;
        L1 = df2 * this.m_s1;
        L2 = df2 * this.m_s2;
        v1.x -= this.m_invMassA * PX;
        v1.y -= this.m_invMassA * PY;
        w1 -= this.m_invIA * L1;
        v2.x += this.m_invMassB * PX;
        v2.y += this.m_invMassB * PY;
        w2 += this.m_invIB * L2;
      }
      bA.m_linearnaHitrost.SetV(v1);
      bA.m_angularVelocity = w1;
      bB.m_linearnaHitrost.SetV(v2);
      bB.m_angularVelocity = w2;
    };
    b2LineJoint.prototype.SolvePositionConstraints = funkcija (baumgarte) {
      if (baumgarte === nedefinirano) baumgarte = 0;
      var limitC = 0;
      var oldLimitImpulse = 0;
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var c1 = bA.m_sweep.c;
      var a1 = bA.m_sweep.a;
      var c2 = bB.m_sweep.c;
      var a2 = bB.m_sweep.a;
      var tMat;
      var tX = 0;
      var m1 = 0;
      var m2 = 0;
      var i1 = 0;
      var i2 = 0;
      var linearError = 0,0;
      var angularError = 0,0;
      var active = false;
      var C2 = 0,0;
      var R1 = b2Mat22.FromAngle(a1);
      var R2 = b2Mat22.FromAngle(a2);
      tMat = R1;
      var r1X = this.m_localAnchor1.x - this.m_localCenterA.x;
      var r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
      tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = R2;
      var r2X = this.m_localAnchor2.x - this.m_localCenterB.x;
      var r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      var dX = c2.x + r2X - c1.x - r1X;
      var dY = c2.y + r2Y - c1.y - r1Y;
      if (this.m_enableLimit) {
        this.m_axis = b2Math.MulMV(R1, this.m_localXAxis1);
        this.m_a1 = (dX + r1X) * this.m_os.y - (dY + r1Y) * this.m_os.x;
        this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
        var prevod = this.m_axis.x * dX + this.m_axis.y * dY;
        če (
          b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) <
          2.0 * b2Settings.b2_linearSlop
        ) {
          C2 = b2Math.Clamp(
            prevod,
            -b2Settings.b2_maxLinearCorrection,
            b2Settings.b2_maxLinearCorrection
          );
          linearError = b2Math.Abs(prevod);
          aktivno = resnično;
        } sicer če (prevod <= this.m_lowerTranslation) {
          C2 = b2Math.Clamp(
            prevod - this.m_lowerTranslation + b2Settings.b2_linearSlop,
            -b2Settings.b2_maxLinearCorrection,
            0,0
          );
          linearError = this.m_lowerTranslation - prevod;
          aktivno = resnično;
        } else if (prevod >= this.m_upperTranslation) {
          C2 = b2Math.Clamp(
            prevod - this.m_upperTranslation + b2Settings.b2_linearSlop,
            0,0,
            b2Settings.b2_maxLinearCorrection
          );
          linearError = prevod - this.m_upperTranslation;
          aktivno = resnično;
        }
      }
      this.m_perp = b2Math.MulMV(R1, this.m_localYAxis1);
      this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
      this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
      var impulz = novo b2Vec2();
      var C1 = this.m_perp.x * dX + this.m_perp.y * dY;
      linearError = b2Math.Max(linearError, b2Math.Abs(C1));
      angularError = 0,0;
      če (aktivno) {
        m1 = this.m_invMassA;
        m2 = this.m_invMassB;
        i1 = this.m_invIA;
        i2 = this.m_invIB;
        this.m_K.col1.x =
          m1 + m2 + i1 * to.m_s1 * to.m_s1 + i2 * to.m_s2 * to.m_s2;
        this.m_K.col1.y = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
        this.m_K.col2.x = this.m_K.col1.y;
        this.m_K.col2.y =
          m1 + m2 + i1 * to.m_a1 * to.m_a1 + i2 * to.m_a2 * to.m_a2;
        this.m_K.Solve(impulz, -C1, -C2);
      } drugače {
        m1 = this.m_invMassA;
        m2 = this.m_invMassB;
        i1 = this.m_invIA;
        i2 = this.m_invIB;
        var k11 =
          m1 + m2 + i1 * to.m_s1 * to.m_s1 + i2 * to.m_s2 * to.m_s2;
        var impulz1 = 0;
        if (k11 != 0,0) {
          impulz1 = -C1 / k11;
        } drugače {
          impulz1 = 0,0;
        }
        impulz.x = impulz1;
        impulz.y = 0,0;
      }
      var PX = impulz.x * this.m_perp.x + impulz.y * this.m_axis.x;
      var PY = impulz.x * this.m_perp.y + impulz.y * this.m_axis.y;
      var L1 = impulz.x * this.m_s1 + impulz.y * this.m_a1;
      var L2 = impulz.x * this.m_s2 + impulz.y * this.m_a2;
      c1.x -= this.m_invMassA * PX;
      c1.y -= this.m_invMassA * PY;
      a1 -= this.m_invIA * L1;
      c2.x += this.m_invMassB * PX;
      c2.y += this.m_invMassB * PY;
      a2 += this.m_invIB * L2;
      bA.m_sweep.a = a1;
      bB.m_sweep.a = a2;
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
      vrnitev (
        linearError <= b2Settings.b2_linearSlop &&
        angularError <= b2Settings.b2_angularSlop
      );
    };
    Box2D.inherit(b2LineJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2LineJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2LineJointDef.b2LineJointDef = funkcija () {
      Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(to, argumenti);
      this.localAnchorA = novo b2Vec2();
      this.localAnchorB = novo b2Vec2();
      this.localAxisA = novo b2Vec2();
    };
    b2LineJointDef.prototype.b2LineJointDef = funkcija () {
      this.__super.b2JointDef.call(this);
      this.type = b2Joint.e_lineJoint;
      this.localAxisA.Set(1.0, 0.0);
      this.enableLimit = false;
      this.lowerTranslation = 0,0;
      this.upperTranslation = 0,0;
      this.enableMotor = false;
      this.maxMotorForce = 0,0;
      this.motorSpeed ​​= 0,0;
    };
    b2LineJointDef.prototype.Initialize = funkcija (bA, bB, sidro, os) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA = this.bodyA.GetLocalPoint(sidro);
      this.localAnchorB = this.bodyB.GetLocalPoint(sidro);
      this.localAxisA = this.bodyA.GetLocalVector(os);
    };
    Box2D.inherit(b2MouseJoint, Box2D.Dynamics.Joints.b2Joint);
    b2MouseJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2MouseJoint.b2MouseJoint = funkcija () {
      Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
      this.K = novo b2Mat22();
      this.K1 = novo b2Mat22();
      this.K2 = novo b2Mat22();
      this.m_localAnchor = novo b2Vec2();
      this.m_target = novo b2Vec2();
      this.m_impulse = novo b2Vec2();
      this.m_mass = novo b2Mat22();
      this.m_C = novo b2Vec2();
    };
    b2MouseJoint.prototype.GetAnchorA = funkcija () {
      vrni this.m_target;
    };
    b2MouseJoint.prototype.GetAnchorB = funkcija () {
      vrni this.m_bodyB.GetWorldPoint(this.m_localAnchor);
    };
    b2MouseJoint.prototype.GetReactionForce = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrni novo b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
    };
    b2MouseJoint.prototype.GetReactionTorque = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrnitev 0,0;
    };
    b2MouseJoint.prototype.GetTarget = funkcija () {
      vrni this.m_target;
    };
    b2MouseJoint.prototype.SetTarget = funkcija (cilj) {
      if (this.m_bodyB.IsAwake() == false) {
        this.m_bodyB.SetAwake(true);
      }
      this.m_target = cilj;
    };
    b2MouseJoint.prototype.GetMaxForce = funkcija () {
      vrni this.m_maxForce;
    };
    b2MouseJoint.prototype.SetMaxForce = funkcija (maxForce) {
      if (maxForce === nedefinirano) maxForce = 0;
      this.m_maxForce = maxForce;
    };
    b2MouseJoint.prototype.GetFrequency = funkcija () {
      vrni this.m_frequencyHz;
    };
    b2MouseJoint.prototype.SetFrequency = funkcija (hz) {
      if (hz === nedefinirano) hz = 0;
      this.m_frekvencaHz = hz;
    };
    b2MouseJoint.prototype.GetDampingRatio = funkcija () {
      vrni this.m_dampingRatio;
    };
    b2MouseJoint.prototype.SetDampingRatio = funkcija (razmerje) {
      če (razmerje === nedefinirano) razmerje = 0;
      this.m_dampingRatio = razmerje;
    };
    b2MouseJoint.prototype.b2MouseJoint = funkcija (def) {
      this.__super.b2Joint.call(this, def);
      this.m_target.SetV(def.target);
      var tX = this.m_target.x - this.m_bodyB.m_xf.position.x;
      var tY = this.m_target.y - this.m_bodyB.m_xf.position.y;
      var tMat = this.m_bodyB.m_xf.R;
      this.m_localAnchor.x = tX * tMat.col1.x + tY * tMat.col1.y;
      this.m_localAnchor.y = tX * tMat.col2.x + tY * tMat.col2.y;
      this.m_maxForce = def.maxForce;
      this.m_impulse.SetZero();
      this.m_frequencyHz = def.frequencyHz;
      this.m_dampingRatio = def.dampingRatio;
      this.m_beta = 0,0;
      this.m_gamma = 0,0;
    };
    b2MouseJoint.prototype.InitVelocityConstraints = funkcija (korak) {
      var b = this.m_bodyB;
      var mass = b.GetMass();
      var omega = 2.0 * Math.PI * this.m_frequencyHz;
      var d = 2,0 * masa * this.m_dampingRatio * omega;
      var k = masa * omega * omega;
      this.m_gamma = step.dt * (d + step.dt * k);
      this.m_gamma = this.m_gamma != 0? 1 / this.m_gamma : 0,0;
      this.m_beta = step.dt * k * this.m_gamma;
      var tMat;
      tMat = b.m_xf.R;
      var rX = this.m_localAnchor.x - b.m_sweep.localCenter.x;
      var rY = this.m_localAnchor.y - b.m_sweep.localCenter.y;
      var tX = tMat.col1.x * rX + tMat.col2.x * rY;
      rY = tMat.col1.y * rX + tMat.col2.y * rY;
      rX = tX;
      var invMass = b.m_invMass;
      var invI = b.m_invI;
      this.K1.col1.x = invMass;
      this.K1.col2.x = 0,0;
      this.K1.col1.y = 0,0;
      this.K1.col2.y = invMass;
      this.K2.col1.x = invI * rY * rY;
      this.K2.col2.x = -invI * rX * rY;
      this.K2.col1.y = -invI * rX * rY;
      this.K2.col2.y = invI * rX * rX;
      this.K.SetM(this.K1);
      to.K.DodajM(ta.K2);
      this.K.col1.x += this.m_gamma;
      this.K.col2.y += this.m_gamma;
      this.K.GetInverse(this.m_mass);
      this.m_C.x = b.m_sweep.cx + rX - this.m_target.x;
      this.m_C.y = b.m_sweep.cy + rY - this.m_target.y;
      b.m_angularVelocity *= 0,98;
      this.m_impulse.x *= step.dtRatio;
      this.m_impulse.y *= step.dtRatio;
      b.m_linearVelocity.x += invMass * this.m_impulse.x;
      b.m_linearVelocity.y += invMass * this.m_impulse.y;
      b.m_angularVelocity +=
        invI * (rX * this.m_impulse.y - rY * this.m_impulse.x);
    };
    b2MouseJoint.prototype.SolveVelocityConstraints = funkcija (korak) {
      var b = this.m_bodyB;
      var tMat;
      var tX = 0;
      var tY = 0;
      tMat = b.m_xf.R;
      var rX = this.m_localAnchor.x - b.m_sweep.localCenter.x;
      var rY = this.m_localAnchor.y - b.m_sweep.localCenter.y;
      tX = tMat.col1.x * rX + tMat.col2.x * rY;
      rY = tMat.col1.y * rX + tMat.col2.y * rY;
      rX = tX;
      var CdotX = b.m_linearVelocity.x + -b.m_angularVelocity * rY;
      var CdotY = b.m_linearVelocity.y + b.m_angularVelocity * rX;
      tMat = this.m_masa;
      tX = CdotX + this.m_beta * this.m_C.x + this.m_gamma * this.m_impulse.x;
      tY = CdotY + this.m_beta * this.m_C.y + this.m_gamma * this.m_impulse.y;
      var impulzX = -(tMat.col1.x * tX + tMat.col2.x * tY);
      var impulzY = -(tMat.col1.y * tX + tMat.col2.y * tY);
      var oldImpulseX = this.m_impulse.x;
      var oldImpulseY = this.m_impulse.y;
      this.m_impulse.x += impulzX;
      this.m_impulse.y += impulzY;
      var maxImpulse = step.dt * this.m_maxForce;
      if (this.m_impulse.LengthSquared() > maxImpulse * maxImpulse) {
        this.m_impulse.Multiply(maxImpulse / this.m_impulse.Length());
      }
      impulzX = this.m_impuls.x - oldImpulseX;
      impulzY = this.m_impulz.y - stari impulzY;
      b.m_linearVelocity.x += b.m_invMass * impulzX;
      b.m_linearVelocity.y += b.m_invMass * impulzY;
      b.m_angularVelocity += b.m_invI * (rX * impulzY - rY * impulzX);
    };
    b2MouseJoint.prototype.SolvePositionConstraints = funkcija (baumgarte) {
      if (baumgarte === nedefinirano) baumgarte = 0;
      vrni resnico;
    };
    Box2D.inherit(b2MouseJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2MouseJointDef.prototype.__super =
      Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2MouseJointDef.b2MouseJointDef = funkcija () {
      Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(to, argumenti);
      this.target = new b2Vec2();
    };
    b2MouseJointDef.prototype.b2MouseJointDef = funkcija () {
      this.__super.b2JointDef.call(this);
      this.type = b2Joint.e_mouseJoint;
      this.maxForce = 0,0;
      this.frequencyHz = 5,0;
      this.dampingRatio = 0,7;
    };
    Box2D.inherit(b2PrismaticJoint, Box2D.Dynamics.Joints.b2Joint);
    b2PrismaticJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2PrismaticJoint.b2PrismaticJoint = funkcija () {
      Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
      this.m_localAnchor1 = novo b2Vec2();
      this.m_localAnchor2 = novo b2Vec2();
      this.m_localXAxis1 = novo b2Vec2();
      this.m_localYAxis1 = novo b2Vec2();
      this.m_axis = new b2Vec2();
      this.m_perp = novo b2Vec2();
      this.m_K = novo b2Mat33();
      this.m_impulse = novo b2Vec3();
    };
    b2PrismaticJoint.prototype.GetAnchorA = funkcija () {
      vrni this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
    };
    b2PrismaticJoint.prototype.GetAnchorB = funkcija () {
      vrni this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
    };
    b2PrismaticJoint.prototype.GetReactionForce = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrni novo b2Vec2(
        inv_dt *
          (this.m_impulse.x * this.m_perp.x +
            (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x),
        inv_dt *
          (this.m_impulse.x * this.m_perp.y +
            (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y)
      );
    };
    b2PrismaticJoint.prototype.GetReactionTorque = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      return inv_dt * this.m_impulse.y;
    };
    b2PrismaticJoint.prototype.GetJointTranslation = funkcija () {
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var tMat;
      var p1 = bA.GetWorldPoint(this.m_localAnchor1);
      var p2 = bB.GetWorldPoint(this.m_localAnchor2);
      var dX = p2.x - p1.x;
      var dY = p2.y - p1.y;
      var axis = bA.GetWorldVector(this.m_localXAxis1);
      var prevod = axis.x * dX + axis.y * dY;
      povratni prevod;
    };
    b2PrismaticJoint.prototype.GetJointSpeed ​​= funkcija () {
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var tMat;
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
      var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      var tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
      var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      var p1X = bA.m_sweep.cx + r1X;
      var p1Y = bA.m_sweep.cy + r1Y;
      var p2X = bB.m_sweep.cx + r2X;
      var p2Y = bB.m_sweep.cy + r2Y;
      var dX = p2X - p1X;
      var dY = p2Y - p1Y;
      var axis = bA.GetWorldVector(this.m_localXAxis1);
      var v1 = bA.m_linearna hitrost;
      var v2 = bB.m_linearVelocity;
      var w1 = bA.m_angularVelocity;
      var w2 = bB.m_angularVelocity;
      varna hitrost =
        dX * (-w1 * os.y) +
        dY * (w1 * os.x) +
        (os.x * (v2.x + -w2 * r2Y - v1.x - -w1 * r1Y) +
          os.y * (v2.y + w2 * r2X - v1.y - w1 * r1X));
      povratna hitrost;
    };
    b2PrismaticJoint.prototype.IsLimitEnabled = funkcija () {
      vrni this.m_enableLimit;
    };
    b2PrismaticJoint.prototype.EnableLimit = funkcija (zastavica) {
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_enableLimit = zastavica;
    };
    b2PrismaticJoint.prototype.GetLowerLimit = funkcija () {
      vrni this.m_lowerTranslation;
    };
    b2PrismaticJoint.prototype.GetUpperLimit = funkcija () {
      vrni this.m_upperTranslation;
    };
    b2PrismaticJoint.prototype.SetLimits = funkcija (spodnji, zgornji) {
      če (spodnji === nedefiniran) nižji = 0;
      če (zgornji === nedefinirano) zgornji = 0;
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_lowerTranslation = nižje;
      this.m_upperTranslation = zgornji;
    };
    b2PrismaticJoint.prototype.IsMotorEnabled = funkcija () {
      vrni this.m_enableMotor;
    };
    b2PrismaticJoint.prototype.EnableMotor = funkcija (zastavica) {
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_enableMotor = zastavica;
    };
    b2PrismaticJoint.prototype.SetMotorSpeed ​​= funkcija (hitrost) {
      if (hitrost === nedefinirano) hitrost = 0;
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_motorSpeed ​​= hitrost;
    };
    b2PrismaticJoint.prototype.GetMotorSpeed ​​= funkcija () {
      vrni this.m_motorSpeed;
    };
    b2PrismaticJoint.prototype.SetMaxMotorForce = funkcija (sila) {
      if (sila === nedefinirano) sila = 0;
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_maxMotorForce = sila;
    };
    b2PrismaticJoint.prototype.GetMotorForce = funkcija () {
      vrni this.m_motorImpulse;
    };
    b2PrismaticJoint.prototype.b2PrismaticJoint = funkcija (def) {
      this.__super.b2Joint.call(this, def);
      var tMat;
      var tX = 0;
      var tY = 0;
      this.m_localAnchor1.SetV(def.localAnchorA);
      this.m_localAnchor2.SetV(def.localAnchorB);
      this.m_localXAxis1.SetV(def.localAxisA);
      this.m_localYAxis1.x = -this.m_localXAxis1.y;
      this.m_localYAxis1.y = this.m_localXAxis1.x;
      this.m_refAngle = def.referenceAngle;
      this.m_impulse.SetZero();
      this.m_motorMass = 0,0;
      this.m_motorImpulse = 0,0;
      this.m_lowerTranslation = def.lowerTranslation;
      this.m_upperTranslation = def.upperTranslation;
      this.m_maxMotorForce = def.maxMotorForce;
      this.m_motorSpeed ​​= def.motorSpeed;
      this.m_enableLimit = def.enableLimit;
      this.m_enableMotor = def.enableMotor;
      this.m_limitState = b2Joint.e_inactiveLimit;
      this.m_axis.SetZero();
      this.m_perp.SetZero();
    };
    b2PrismaticJoint.prototype.InitVelocityConstraints = funkcija (korak) {
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var tMat;
      var tX = 0;
      this.m_localCenterA.SetV(bA.GetLocalCenter());
      this.m_localCenterB.SetV(bB.GetLocalCenter());
      var xf1 = bA.GetTransform();
      var xf2 = bB.GetTransform();
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - this.m_localCenterA.x;
      var r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
      tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - this.m_localCenterB.x;
      var r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      var dX = bB.m_sweep.cx + r2X - bA.m_sweep.cx - r1X;
      var dY = bB.m_sweep.cy + r2Y - bA.m_sweep.cy - r1Y;
      this.m_invMassA = bA.m_invMass;
      this.m_invMassB = bB.m_invMass;
      this.m_invIA = bA.m_invI;
      this.m_invIB = bB.m_invI;
      {
        this.m_axis.SetV(b2Math.MulMV(xf1.R, this.m_localXAxis1));
        this.m_a1 = (dX + r1X) * this.m_os.y - (dY + r1Y) * this.m_os.x;
        this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
        this.m_motorMass =
          this.m_invMassA +
          this.m_invMassB +
          this.m_invIA * this.m_a1 * this.m_a1 +
          this.m_invIB * this.m_a2 * this.m_a2;
        if (this.m_motorMass > Number.MIN_VALUE)
          this.m_motorMass = 1,0 / this.m_motorMass;
      }
      {
        this.m_perp.SetV(b2Math.MulMV(xf1.R, this.m_localYAxis1));
        this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
        this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
        var m1 = this.m_invMassA;
        var m2 = this.m_invMassB;
        var i1 = this.m_invIA;
        var i2 = this.m_invIB;
        this.m_K.col1.x =
          m1 + m2 + i1 * to.m_s1 * to.m_s1 + i2 * to.m_s2 * to.m_s2;
        this.m_K.col1.y = i1 * this.m_s1 + i2 * this.m_s2;
        this.m_K.col1.z = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
        this.m_K.col2.x = this.m_K.col1.y;
        this.m_K.col2.y = i1 + i2;
        this.m_K.col2.z = i1 * this.m_a1 + i2 * this.m_a2;
        this.m_K.col3.x = this.m_K.col1.z;
        this.m_K.col3.y = this.m_K.col2.z;
        this.m_K.col3.z =
          m1 + m2 + i1 * to.m_a1 * to.m_a1 + i2 * to.m_a2 * to.m_a2;
      }
      if (this.m_enableLimit) {
        var jointTransition = this.m_axis.x * dX + this.m_axis.y * dY;
        če (
          b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) <
          2.0 * b2Settings.b2_linearSlop
        ) {
          this.m_limitState = b2Joint.e_equalLimits;
        } else if (jointTransition <= this.m_lowerTranslation) {
          if (this.m_limitState != b2Joint.e_atLowerLimit) {
            this.m_limitState = b2Joint.e_atLowerLimit;
            this.m_impulse.z = 0,0;
          }
        } else if (jointTransition >= this.m_upperTranslation) {
          if (this.m_limitState != b2Joint.e_atUpperLimit) {
            this.m_limitState = b2Joint.e_atUpperLimit;
            this.m_impulse.z = 0,0;
          }
        } drugače {
          this.m_limitState = b2Joint.e_inactiveLimit;
          this.m_impulse.z = 0,0;
        }
      } drugače {
        this.m_limitState = b2Joint.e_inactiveLimit;
      }
      if (this.m_enableMotor == false) {
        this.m_motorImpulse = 0,0;
      }
      if (step.warmStarting) {
        this.m_impulse.x *= step.dtRatio;
        this.m_impulse.y *= step.dtRatio;
        this.m_motorImpulse *= step.dtRatio;
        var PX =
          this.m_impulse.x * this.m_perp.x +
          (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x;
        var PY =
          this.m_impulse.x * this.m_perp.y +
          (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y;
        var L1 =
          this.m_impulse.x * this.m_s1 +
          this.m_impulse.y +
          (this.m_motorImpulse + this.m_impulse.z) * this.m_a1;
        var L2 =
          this.m_impulse.x * this.m_s2 +
          this.m_impulse.y +
          (this.m_motorImpulse + this.m_impulse.z) * this.m_a2;
        bA.m_linearVelocity.x -= this.m_invMassA * PX;
        bA.m_linearVelocity.y -= this.m_invMassA * PY;
        bA.m_angularVelocity -= this.m_invIA * L1;
        bB.m_linearVelocity.x += this.m_invMassB * PX;
        bB.m_linearVelocity.y += this.m_invMassB * PY;
        bB.m_angularVelocity += this.m_invIB * L2;
      } drugače {
        this.m_impulse.SetZero();
        this.m_motorImpulse = 0,0;
      }
    };
    b2PrismaticJoint.prototype.SolveVelocityConstraints = funkcija (korak) {
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var v1 = bA.m_linearna hitrost;
      var w1 = bA.m_angularVelocity;
      var v2 = bB.m_linearVelocity;
      var w2 = bB.m_angularVelocity;
      var PX = 0;
      var. PY = 0;
      var L1 = 0;
      var L2 = 0;
      if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits) {
        var Cdot =
          this.m_axis.x * (v2.x - v1.x) +
          this.m_axis.y * (v2.y - v1.y) +
          to.m_a2 * w2 -
          this.m_a1 * w1;
        var impulz = this.m_motorMass * (this.m_motorSpeed ​​- Cdot);
        var oldImpulse = this.m_motorImpulse;
        var maxImpulse = step.dt * this.m_maxMotorForce;
        this.m_motorImpulse = b2Math.Clamp(
          this.m_motorImpulse + impulz,
          -največji impulz,
          maxImpulse
        );
        impulz = this.m_motorImpulse - stari impulz;
        PX = impulz * this.m_axis.x;
        PY = impulz * this.m_axis.y;
        L1 = impulz * this.m_a1;
        L2 = impulz * this.m_a2;
        v1.x -= this.m_invMassA * PX;
        v1.y -= this.m_invMassA * PY;
        w1 -= this.m_invIA * L1;
        v2.x += this.m_invMassB * PX;
        v2.y += this.m_invMassB * PY;
        w2 += this.m_invIB * L2;
      }
      var Cdot1X =
        this.m_perp.x * (v2.x - v1.x) +
        this.m_perp.y * (v2.y - v1.y) +
        this.m_s2 * w2 -
        this.m_s1 * w1;
      var Cdot1Y = w2 - w1;
      if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
        var Cdot2 =
          this.m_axis.x * (v2.x - v1.x) +
          this.m_axis.y * (v2.y - v1.y) +
          to.m_a2 * w2 -
          this.m_a1 * w1;
        var f1 = this.m_impulse.Copy();
        var df = this.m_K.Solve33(novo b2Vec3(), -Cdot1X, -Cdot1Y, -Cdot2);
        this.m_impulse.Add(df);
        if (this.m_limitState == b2Joint.e_atLowerLimit) {
          this.m_impulse.z = b2Math.Max(this.m_impulse.z, 0,0);
        } else if (this.m_limitState == b2Joint.e_atUpperLimit) {
          this.m_impulse.z = b2Math.Min(this.m_impulse.z, 0,0);
        }
        var bX = -Cdot1X - (this.m_impulse.z - f1.z) * this.m_K.col3.x;
        var bY = -Cdot1Y - (this.m_impulse.z - f1.z) * this.m_K.col3.y;
        var f2r = this.m_K.Solve22(novo b2Vec2(), bX, bY);
        f2r.x += f1.x;
        f2r.y += f1.y;
        this.m_impulse.x = f2r.x;
        this.m_impulse.y = f2r.y;
        df.x = this.m_impulse.x - f1.x;
        df.y = ta.m_impulz.y - f1.y;
        df.z = ta.m_impulz.z - f1.z;
        PX = df.x * this.m_perp.x + df.z * this.m_axis.x;
        PY = df.x * this.m_perp.y + df.z * this.m_axis.y;
        L1 = df.x * this.m_s1 + df.y + df.z * this.m_a1;
        L2 = df.x * this.m_s2 + df.y + df.z * this.m_a2;
        v1.x -= this.m_invMassA * PX;
        v1.y -= this.m_invMassA * PY;
        w1 -= this.m_invIA * L1;
        v2.x += this.m_invMassB * PX;
        v2.y += this.m_invMassB * PY;
        w2 += this.m_invIB * L2;
      } drugače {
        var df2 = this.m_K.Solve22(novo b2Vec2(), -Cdot1X, -Cdot1Y);
        this.m_impulse.x += df2.x;
        this.m_impulse.y += df2.y;
        PX = df2.x * this.m_perp.x;
        PY = df2.x * this.m_perp.y;
        L1 = df2.x * this.m_s1 + df2.y;
        L2 = df2.x * this.m_s2 + df2.y;
        v1.x -= this.m_invMassA * PX;
        v1.y -= this.m_invMassA * PY;
        w1 -= this.m_invIA * L1;
        v2.x += this.m_invMassB * PX;
        v2.y += this.m_invMassB * PY;
        w2 += this.m_invIB * L2;
      }
      bA.m_linearnaHitrost.SetV(v1);
      bA.m_angularVelocity = w1;
      bB.m_linearnaHitrost.SetV(v2);
      bB.m_angularVelocity = w2;
    };
    b2PrismaticJoint.prototype.SolvePositionConstraints = funkcija (baumgarte) {
      if (baumgarte === nedefinirano) baumgarte = 0;
      var limitC = 0;
      var oldLimitImpulse = 0;
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var c1 = bA.m_sweep.c;
      var a1 = bA.m_sweep.a;
      var c2 = bB.m_sweep.c;
      var a2 = bB.m_sweep.a;
      var tMat;
      var tX = 0;
      var m1 = 0;
      var m2 = 0;
      var i1 = 0;
      var i2 = 0;
      var linearError = 0,0;
      var angularError = 0,0;
      var active = false;
      var C2 = 0,0;
      var R1 = b2Mat22.FromAngle(a1);
      var R2 = b2Mat22.FromAngle(a2);
      tMat = R1;
      var r1X = this.m_localAnchor1.x - this.m_localCenterA.x;
      var r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
      tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = R2;
      var r2X = this.m_localAnchor2.x - this.m_localCenterB.x;
      var r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      var dX = c2.x + r2X - c1.x - r1X;
      var dY = c2.y + r2Y - c1.y - r1Y;
      if (this.m_enableLimit) {
        this.m_axis = b2Math.MulMV(R1, this.m_localXAxis1);
        this.m_a1 = (dX + r1X) * this.m_os.y - (dY + r1Y) * this.m_os.x;
        this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
        var prevod = this.m_axis.x * dX + this.m_axis.y * dY;
        če (
          b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) <
          2.0 * b2Settings.b2_linearSlop
        ) {
          C2 = b2Math.Clamp(
            prevod,
            -b2Settings.b2_maxLinearCorrection,
            b2Settings.b2_maxLinearCorrection
          );
          linearError = b2Math.Abs(prevod);
          aktivno = resnično;
        } sicer če (prevod <= this.m_lowerTranslation) {
          C2 = b2Math.Clamp(
            prevod - this.m_lowerTranslation + b2Settings.b2_linearSlop,
            -b2Settings.b2_maxLinearCorrection,
            0,0
          );
          linearError = this.m_lowerTranslation - prevod;
          aktivno = resnično;
        } else if (prevod >= this.m_upperTranslation) {
          C2 = b2Math.Clamp(
            prevod - this.m_upperTranslation + b2Settings.b2_linearSlop,
            0,0,
            b2Settings.b2_maxLinearCorrection
          );
          linearError = prevod - this.m_upperTranslation;
          aktivno = resnično;
        }
      }
      this.m_perp = b2Math.MulMV(R1, this.m_localYAxis1);
      this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
      this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
      var impulz = novo b2Vec3();
      var C1X = this.m_perp.x * dX + this.m_perp.y * dY;
      var C1Y = a2 - a1 - this.m_refAngle;
      linearError = b2Math.Max(linearError, b2Math.Abs(C1X));
      angularError = b2Math.Abs(C1Y);
      če (aktivno) {
        m1 = this.m_invMassA;
        m2 = this.m_invMassB;
        i1 = this.m_invIA;
        i2 = this.m_invIB;
        this.m_K.col1.x =
          m1 + m2 + i1 * to.m_s1 * to.m_s1 + i2 * to.m_s2 * to.m_s2;
        this.m_K.col1.y = i1 * this.m_s1 + i2 * this.m_s2;
        this.m_K.col1.z = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
        this.m_K.col2.x = this.m_K.col1.y;
        this.m_K.col2.y = i1 + i2;
        this.m_K.col2.z = i1 * this.m_a1 + i2 * this.m_a2;
        this.m_K.col3.x = this.m_K.col1.z;
        this.m_K.col3.y = this.m_K.col2.z;
        this.m_K.col3.z =
          m1 + m2 + i1 * to.m_a1 * to.m_a1 + i2 * to.m_a2 * to.m_a2;
        this.m_K.Solve33(impulz, -C1X, -C1Y, -C2);
      } drugače {
        m1 = this.m_invMassA;
        m2 = this.m_invMassB;
        i1 = this.m_invIA;
        i2 = this.m_invIB;
        var k11 =
          m1 + m2 + i1 * to.m_s1 * to.m_s1 + i2 * to.m_s2 * to.m_s2;
        var k12 = i1 * this.m_s1 + i2 * this.m_s2;
        var k22 = i1 + i2;
        this.m_K.col1.Set(k11, k12, 0,0);
        this.m_K.col2.Set(k12, k22, 0,0);
        var impulz1 = this.m_K.Solve22(novo b2Vec2(), -C1X, -C1Y);
        impulz.x = impulz1.x;
        impulz.y = impulz1.y;
        impulz.z = 0,0;
      }
      var PX = impulz.x * this.m_perp.x + impulz.z * this.m_axis.x;
      var PY = impulz.x * this.m_perp.y + impulz.z * this.m_axis.y;
      var L1 = impulz.x * this.m_s1 + impulz.y + impulz.z * this.m_a1;
      var L2 = impulz.x * this.m_s2 + impulz.y + impulz.z * this.m_a2;
      c1.x -= this.m_invMassA * PX;
      c1.y -= this.m_invMassA * PY;
      a1 -= this.m_invIA * L1;
      c2.x += this.m_invMassB * PX;
      c2.y += this.m_invMassB * PY;
      a2 += this.m_invIB * L2;
      bA.m_sweep.a = a1;
      bB.m_sweep.a = a2;
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
      vrnitev (
        linearError <= b2Settings.b2_linearSlop &&
        angularError <= b2Settings.b2_angularSlop
      );
    };
    Box2D.inherit(b2PrismaticJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2PrismaticJointDef.prototype.__super =
      Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2PrismaticJointDef.b2PrismaticJointDef = funkcija () {
      Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(to, argumenti);
      this.localAnchorA = novo b2Vec2();
      this.localAnchorB = novo b2Vec2();
      this.localAxisA = novo b2Vec2();
    };
    b2PrismaticJointDef.prototype.b2PrismaticJointDef = funkcija () {
      this.__super.b2JointDef.call(this);
      this.type = b2Joint.e_prismaticJoint;
      this.localAxisA.Set(1.0, 0.0);
      this.referenceAngle = 0,0;
      this.enableLimit = false;
      this.lowerTranslation = 0,0;
      this.upperTranslation = 0,0;
      this.enableMotor = false;
      this.maxMotorForce = 0,0;
      this.motorSpeed ​​= 0,0;
    };
    b2PrismaticJointDef.prototype.Initialize = funkcija (bA, bB, sidro, os) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA = this.bodyA.GetLocalPoint(sidro);
      this.localAnchorB = this.bodyB.GetLocalPoint(sidro);
      this.localAxisA = this.bodyA.GetLocalVector(os);
      this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    };
    Box2D.inherit(b2PulleyJoint, Box2D.Dynamics.Joints.b2Joint);
    b2PulleyJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2PulleyJoint.b2PulleyJoint = funkcija () {
      Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
      this.m_groundAnchor1 = novo b2Vec2();
      this.m_groundAnchor2 = novo b2Vec2();
      this.m_localAnchor1 = novo b2Vec2();
      this.m_localAnchor2 = novo b2Vec2();
      this.m_u1 = novo b2Vec2();
      this.m_u2 = novo b2Vec2();
    };
    b2PulleyJoint.prototype.GetAnchorA = funkcija () {
      vrni this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
    };
    b2PulleyJoint.prototype.GetAnchorB = funkcija () {
      vrni this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
    };
    b2PulleyJoint.prototype.GetReactionForce = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrni novo b2Vec2(
        inv_dt * this.m_impulse * this.m_u2.x,
        inv_dt * this.m_impulse * this.m_u2.y
      );
    };
    b2PulleyJoint.prototype.GetReactionTorque = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrnitev 0,0;
    };
    b2PulleyJoint.prototype.GetGroundAnchorA = funkcija () {
      var a = this.m_ground.m_xf.position.Copy();
      a.Add(this.m_groundAnchor1);
      vrnitev a;
    };
    b2PulleyJoint.prototype.GetGroundAnchorB = funkcija () {
      var a = this.m_ground.m_xf.position.Copy();
      a.Add(this.m_groundAnchor2);
      vrnitev a;
    };
    b2PulleyJoint.prototype.GetLength1 = funkcija () {
      var p = this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
      var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
      var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
      var dX = px - sX;
      var dY = py - sY;
      return Math.sqrt(dX * dX + dY * dY);
    };
    b2PulleyJoint.prototype.GetLength2 = funkcija () {
      var p = this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
      var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
      var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
      var dX = px - sX;
      var dY = py - sY;
      return Math.sqrt(dX * dX + dY * dY);
    };
    b2PulleyJoint.prototype.GetRatio = funkcija () {
      vrni this.m_ratio;
    };
    b2PulleyJoint.prototype.b2PulleyJoint = funkcija (def) {
      this.__super.b2Joint.call(this, def);
      var tMat;
      var tX = 0;
      var tY = 0;
      this.m_ground = this.m_bodyA.m_world.m_groundBody;
      this.m_groundAnchor1.x =
        def.groundAnchorA.x - this.m_ground.m_xf.position.x;
      this.m_groundAnchor1.y =
        def.groundAnchorA.y - this.m_ground.m_xf.position.y;
      this.m_groundAnchor2.x =
        def.groundAnchorB.x - this.m_ground.m_xf.position.x;
      this.m_groundAnchor2.y =
        def.groundAnchorB.y - this.m_ground.m_xf.position.y;
      this.m_localAnchor1.SetV(def.localAnchorA);
      this.m_localAnchor2.SetV(def.localAnchorB);
      this.m_ratio = def.ratio;
      this.m_constant = def.lengthA + this.m_ratio * def.lengthB;
      this.m_maxLength1 = b2Math.Min(
        def.maxLengthA,
        this.m_constant - this.m_ratio * b2PulleyJoint.b2_minPulleyLength
      );
      this.m_maxLength2 = b2Math.Min(
        def.maxLengthB,
        (this.m_constant - b2PulleyJoint.b2_minPulleyLength) / this.m_ratio
      );
      this.m_impulz = 0,0;
      this.m_limitImpulse1 = 0,0;
      this.m_limitImpulse2 = 0,0;
    };
    b2PulleyJoint.prototype.InitVelocityConstraints = funkcija (korak) {
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var tMat;
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
      var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      var tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
      var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      var p1X = bA.m_sweep.cx + r1X;
      var p1Y = bA.m_sweep.cy + r1Y;
      var p2X = bB.m_sweep.cx + r2X;
      var p2Y = bB.m_sweep.cy + r2Y;
      var s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
      var s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
      var s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
      var s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
      this.m_u1.Set(p1X - s1X, p1Y - s1Y);
      this.m_u2.Set(p2X - s2X, p2Y - s2Y);
      var length1 = this.m_u1.Length();
      var length2 = this.m_u2.Length();
      if (length1 > b2Settings.b2_linearSlop) {
        this.m_u1.Množi(1,0 / dolžina1);
      } drugače {
        this.m_u1.SetZero();
      }
      if (length2 > b2Settings.b2_linearSlop) {
        this.m_u2.Množi (1,0 / dolžina2);
      } drugače {
        this.m_u2.SetZero();
      }
      var C = this.m_constant - length1 - this.m_ratio * length2;
      če (C > 0,0) {
        this.m_state = b2Joint.e_inactiveLimit;
        this.m_impulz = 0,0;
      } drugače {
        this.m_state = b2Joint.e_atUpperLimit;
      }
      if (length1 < this.m_maxLength1) {
        this.m_limitState1 = b2Joint.e_inactiveLimit;
        this.m_limitImpulse1 = 0,0;
      } drugače {
        this.m_limitState1 = b2Joint.e_atUpperLimit;
      }
      if (length2 < this.m_maxLength2) {
        this.m_limitSta te2 = b2Joint.e_inactiveLimit;
        this.m_limitImpulse2 = 0,0;
      } drugače {
        this.m_limitState2 = b2Joint.e_atUpperLimit;
      }
      var cr1u1 = r1X * this.m_u1.y - r1Y * this.m_u1.x;
      var cr2u2 = r2X * this.m_u2.y - r2Y * this.m_u2.x;
      this.m_limitMass1 = bA.m_invMass + bA.m_invI * cr1u1 * cr1u1;
      this.m_limitMass2 = bB.m_invMass + bB.m_invI * cr2u2 * cr2u2;
      this.m_pulleyMass =
        this.m_limitMass1 + this.m_ratio * this.m_ratio * this.m_limitMass2;
      this.m_limitMass1 = 1,0 / this.m_limitMass1;
      this.m_limitMass2 = 1,0 / this.m_limitMass2;
      this.m_pulleyMass = 1,0 / this.m_pulleyMass;
      if (step.warmStarting) {
        this.m_impulse *= step.dtRatio;
        this.m_limitImpulse1 *= step.dtRatio;
        this.m_limitImpulse2 *= step.dtRatio;
        var P1X = (-this.m_impulse - this.m_limitImpulse1) * this.m_u1.x;
        var P1Y = (-this.m_impulse - this.m_limitImpulse1) * this.m_u1.y;
        var P2X =
          (-this.m_ratio * this.m_impulse - this.m_limitImpulse2) * this.m_u2.x;
        var P2Y =
          (-this.m_ratio * this.m_impulse - this.m_limitImpulse2) * this.m_u2.y;
        bA.m_linearVelocity.x += bA.m_invMass * P1X;
        bA.m_linearVelocity.y += bA.m_invMasa * P1Y;
        bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
        bB.m_linearVelocity.x += bB.m_invMass * P2X;
        bB.m_linearVelocity.y += bB.m_invMass * P2Y;
        bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
      } drugače {
        this.m_impulz = 0,0;
        this.m_limitImpulse1 = 0,0;
        this.m_limitImpulse2 = 0,0;
      }
    };
    b2PulleyJoint.prototype.SolveVelocityConstraints = funkcija (korak) {
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var tMat;
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
      var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      var tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
      var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      var v1X = 0;
      var v1Y = 0;
      var v2X = 0;
      var v2Y = 0;
      var P1X = 0;
      var. P1Y = 0;
      var P2X = 0;
      var P2Y = 0;
      var Cdot = 0;
      var impulz = 0;
      var oldImpulse = 0;
      if (this.m_state == b2Joint.e_atUpperLimit) {
        v1X = bA.m_linearnaHitrost.x + -bA.m_kotnaHitrost * r1Y;
        v1Y = bA.m_linearna hitrost.y + bA.m_kotna hitrost * r1X;
        v2X = bB.m_linearnaHitrost.x + -bB.m_kotnaHitrost * r2Y;
        v2Y = bB.m_linearnaHitrost.y + bB.m_kotnaHitrost * r2X;
        Cdot =
          -(this.m_u1.x * v1X + this.m_u1.y * v1Y) -
          this.m_ratio * (this.m_u2.x * v2X + this.m_u2.y * v2Y);
        impulz = this.m_pulleyMass * -Cdot;
        oldImpulse = this.m_impulse;
        this.m_impulse = b2Math.Max(0,0, this.m_impulse + impulz);
        impulz = this.m_impuls - stari impulz;
        P1X = -impulz * this.m_u1.x;
        P1Y = -impulz * this.m_u1.y;
        P2X = -this.m_ratio * impulz * this.m_u2.x;
        P2Y = -this.m_racio * impulz * this.m_u2.y;
        bA.m_linearVelocity.x += bA.m_invMass * P1X;
        bA.m_linearVelocity.y += bA.m_invMasa * P1Y;
        bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
        bB.m_linearVelocity.x += bB.m_invMass * P2X;
        bB.m_linearVelocity.y += bB.m_invMass * P2Y;
        bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
      }
      if (this.m_limitState1 == b2Joint.e_atUpperLimit) {
        v1X = bA.m_linearnaHitrost.x + -bA.m_kotnaHitrost * r1Y;
        v1Y = bA.m_linearna hitrost.y + bA.m_kotna hitrost * r1X;
        Cdot = -(this.m_u1.x * v1X + this.m_u1.y * v1Y);
        impulz = -this.m_limitMass1 * Cdot;
        oldImpulse = this.m_limitImpulse1;
        this.m_limitImpulse1 = b2Math.Max(0,0, this.m_limitImpulse1 + impulz);
        impulz = this.m_limitImpulse1 - oldImpulse;
        P1X = -impulz * this.m_u1.x;
        P1Y = -impulz * this.m_u1.y;
        bA.m_linearVelocity.x += bA.m_invMass * P1X;
        bA.m_linearVelocity.y += bA.m_invMasa * P1Y;
        bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
      }
      if (this.m_limitState2 == b2Joint.e_atUpperLimit) {
        v2X = bB.m_linearnaHitrost.x + -bB.m_kotnaHitrost * r2Y;
        v2Y = bB.m_linearnaHitrost.y + bB.m_kotnaHitrost * r2X;
        Cdot = -(this.m_u2.x * v2X + this.m_u2.y * v2Y);
        impulz = -this.m_limitMass2 * Cdot;
        oldImpulse = this.m_limitImpulse2;
        this.m_limitImpulse2 = b2Math.Max(0,0, this.m_limitImpulse2 + impulz);
        impulz = this.m_limitImpulse2 - oldImpulse;
        P2X = -impulz * this.m_u2.x;
        P2Y = -impulz * this.m_u2.y;
        bB.m_linearVelocity.x += bB.m_invMass * P2X;
        bB.m_linearVelocity.y += bB.m_invMass * P2Y;
        bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
      }
    };
    b2PulleyJoint.prototype.SolvePositionConstraints = funkcija (baumgarte) {
      if (baumgarte === nedefinirano) baumgarte = 0;
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var tMat;
      var s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
      var s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
      var s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
      var s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
      var r1X = 0;
      var r1Y = 0;
      var r2X = 0;
      var r2Y = 0;
      var p1X = 0;
      var p1Y = 0;
      var p2X = 0;
      var p2Y = 0;
      spremenljiva dolžina1 = 0;
      spremenljiva dolžina2 = 0;
      var C = 0;
      var impulz = 0;
      var oldImpulse = 0;
      var oldLimitPositionImpulse = 0;
      var tX = 0;
      var linearError = 0,0;
      if (this.m_state == b2Joint.e_atUpperLimit) {
        tMat = bA.m_xf.R;
        r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
        r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
        tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
        r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
        r1X = tX;
        tMat = bB.m_xf.R;
        r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
        r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
        tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
        r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
        r2X = tX;
        p1X = bA.m_sweep.cx + r1X;
        p1Y = bA.m_sweep.cy + r1Y;
        p2X = bB.m_sweep.cx + r2X;
        p2Y = bB.m_sweep.cy + r2Y;
        this.m_u1.Set(p1X - s1X, p1Y - s1Y);
        this.m_u2.Set(p2X - s2X, p2Y - s2Y);
        dolžina1 = this.m_u1.Length();
        dolžina2 = this.m_u2.Length();
        if (length1 > b2Settings.b2_linearSlop) {
          this.m_u1.Množi(1,0 / dolžina1);
        } drugače {
          this.m_u1.SetZero();
        }
        if (length2 > b2Settings.b2_linearSlop) {
          this.m_u2.Množi (1,0 / dolžina2);
        } drugače {
          this.m_u2.SetZero();
        }
        C = ta.m_konstanta - dolžina1 - to.m_razmerje * dolžina2;
        linearError = b2Math.Max(linearError, -C);
        C = b2Math.Clamp(
          C + b2Settings.b2_linearSlop,
          -b2Settings.b2_maxLinearCorrection,
          0,0
        );
        impulz = -this.m_pulleyMass * C;
        p1X = -impulz * this.m_u1.x;
        p1Y = -impulz * this.m_u1.y;
        p2X = -this.m_ratio * impulz * this.m_u2.x;
        p2Y = -this.m_racio * impulz * this.m_u2.y;
        bA.m_sweep.cx += bA.m_invMass * p1X;
        bA.m_sweep.cy += bA.m_invMass * p1Y;
        bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);
        bB.m_sweep.cx += bB.m_invMass * p2X;
        bB.m_sweep.cy += bB.m_invMass * p2Y;
        bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);
        bA.SynchronizeTransform();
        bB.SynchronizeTransform();
      }
      if (this.m_limitState1 == b2Joint.e_atUpperLimit) {
        tMat = bA.m_xf.R;
        r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
        r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
        tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
        r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
        r1X = tX;
        p1X = bA.m_sweep.cx + r1X;
        p1Y = bA.m_sweep.cy + r1Y;
        this.m_u1.Set(p1X - s1X, p1Y - s1Y);
        dolžina1 = this.m_u1.Length();
        if (length1 > b2Settings.b2_linearSlop) {
          this.m_u1.x *= 1,0 / dolžina1;
          this.m_u1.y *= 1,0 / dolžina1;
        } drugače {
          this.m_u1.SetZero();
        }
        C = this.m_maxLength1 - dolžina1;
        linearError = b2Math.Max(linearError, -C);
        C = b2Math.Clamp(
          C + b2Settings.b2_linearSlop,
          -b2Settings.b2_maxLinearCorrection,
          0,0
        );
        impulz = -this.m_limitMass1 * C;
        p1X = -impulz * this.m_u1.x;
        p1Y = -impulz * this.m_u1.y;
        bA.m_sweep.cx += bA.m_invMass * p1X;
        bA.m_sweep.cy += bA.m_invMass * p1Y;
        bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);
        bA.SynchronizeTransform();
      }
      if (this.m_limitState2 == b2Joint.e_atUpperLimit) {
        tMat = bB.m_xf.R;
        r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
        r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
        tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
        r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
        r2X = tX;
        p2X = bB.m_sweep.cx + r2X;
        p2Y = bB.m_sweep.cy + r2Y;
        this.m_u2.Set(p2X - s2X, p2Y - s2Y);
        dolžina2 = this.m_u2.Length();
        if (length2 > b2Settings.b2_linearSlop) {
          this.m_u2.x *= 1,0 / dolžina2;
          this.m_u2.y *= 1,0 / dolžina2;
        } drugače {
          this.m_u2.SetZero();
        }
        C = this.m_maxLength2 - dolžina2;
        linearError = b2Math.Max(linearError, -C);
        C = b2Math.Clamp(
          C + b2Settings.b2_linearSlop,
          -b2Settings.b2_maxLinearCorrection,
          0,0
        );
        impulz = -this.m_limitMass2 * C;
        p2X = -impulz * this.m_u2.x;
        p2Y = -impulz * this.m_u2.y;
        bB.m_sweep.cx += bB.m_invMass * p2X;
        bB.m_sweep.cy += bB.m_invMass * p2Y;
        bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);
        bB.SynchronizeTransform();
      }
      return linearError < b2Settings.b2_linearSlop;
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Dynamics.Joints.b2PulleyJoint.b2_minPulleyLength = 2,0;
    });
    Box2D.inherit(b2PulleyJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2PulleyJointDef.prototype.__super =
      Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2PulleyJointDef.b2PulleyJointDef = funkcija () {
      Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(to, argumenti);
      this.groundAnchorA = novo b2Vec2();
      this.groundAnchorB = novo b2Vec2();
      this.localAnchorA = novo b2Vec2();
      this.localAnchorB = novo b2Vec2();
    };
    b2PulleyJointDef.prototype.b2PulleyJointDef = funkcija () {
      this.__super.b2JointDef.call(this);
      this.type = b2Joint.e_pulleyJoint;
      this.groundAnchorA.Set(-1.0, 1.0);
      this.groundAnchorB.Set(1.0, 1.0);
      this.localAnchorA.Set(-1.0, 0.0);
      this.localAnchorB.Set(1.0, 0.0);
      this.lengthA = 0,0;
      this.maxLengthA = 0,0;
      this.lengthB = 0,0;
      this.maxLengthB = 0,0;
      this.ratio = 1,0;
      this.collideConnected = res;
    };
    b2PulleyJointDef.prototype.Initialize = funkcija (
      bA,
      bB,
      gaA,
      gaB,
      sidroA,
      sidroB,
      r
    ) {
      if (r === nedefinirano) r = 0;
      this.bodyA = bA;
      this.bodyB = bB;
      this.groundAnchorA.SetV(gaA);
      this.groundAnchorB.SetV(gaB);
      this.localAnchorA = this.bodyA.GetLocalPoint(anchorA);
      this.localAnchorB = this.bodyB.GetLocalPoint(anchorB);
      var d1X = sidro A.x - gaA.x;
      var d1Y = sidro A.y - gaA.y;
      this.lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y);
      var d2X = sidroB.x - gaB.x;
      var d2Y = anchorB.y - gaB.y;
      this.lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y);
      to.razmerje = r;
      var C = this.lengthA + this.ratio * this.lengthB;
      this.maxLengthA = C - this.ratio * b2PulleyJoint.b2_minPulleyLength;
      this.maxLengthB = (C - b2PulleyJoint.b2_minPulleyLength) / this.ratio;
    };
    Box2D.inherit(b2RevoluteJoint, Box2D.Dynamics.Joints.b2Joint);
    b2RevoluteJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2RevoluteJoint.b2RevoluteJoint = funkcija () {
      Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
      this.K = novo b2Mat22();
      this.K1 = novo b2Mat22();
      this.K2 = novo b2Mat22();
      this.K3 = novo b2Mat22();
      this.impulse3 = novo b2Vec3();
      this.impulse2 = novo b2Vec2();
      this.reduced = new b2Vec2();
      this.m_localAnchor1 = novo b2Vec2();
      this.m_localAnchor2 = novo b2Vec2();
      this.m_impulse = novo b2Vec3();
      this.m_mass = novo b2Mat33();
    };
    b2RevoluteJoint.prototype.GetAnchorA = funkcija () {
      vrni this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
    };
    b2RevoluteJoint.prototype.GetAnchorB = funkcija () {
      vrni this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
    };
    b2RevoluteJoint.prototype.GetReactionForce = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrni novo b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
    };
    b2RevoluteJoint.prototype.GetReactionTorque = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      return inv_dt * this.m_impulse.z;
    };
    b2RevoluteJoint.prototype.GetJointAngle = funkcija () {
      vrnitev (
        this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle
      );
    };
    b2RevoluteJoint.prototype.GetJointSpeed ​​= funkcija () {
      vrni this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity;
    };
    b2RevoluteJoint.prototype.IsLimitEnabled = funkcija () {
      vrni this.m_enableLimit;
    };
    b2RevoluteJoint.prototype.EnableLimit = funkcija (zastavica) {
      this.m_enableLimit = zastavica;
    };
    b2RevoluteJoint.prototype.GetLowerLimit = funkcija () {
      vrni this.m_lowerAngle;
    };
    b2RevoluteJoint.prototype.GetUpperLimit = funkcija () {
      vrni this.m_upperAngle;
    };
    b2RevoluteJoint.prototype.SetLimits = funkcija (spodnji, zgornji) {
      če (spodnji === nedefiniran) nižji = 0;
      če (zgornji === nedefinirano) zgornji = 0;
      this.m_lowerAngle = nižji;
      this.m_upperAngle = zgornji;
    };
    b2RevoluteJoint.prototype.IsMotorEnabled = funkcija () {
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      vrni this.m_enableMotor;
    };
    b2RevoluteJoint.prototype.EnableMotor = funkcija (zastavica) {
      this.m_enableMotor = zastavica;
    };
    b2RevoluteJoint.prototype.SetMotorSpeed ​​= funkcija (hitrost) {
      if (hitrost === nedefinirano) hitrost = 0;
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_motorSpeed ​​= hitrost;
    };
    b2RevoluteJoint.prototype.GetMotorSpeed ​​= funkcija () {
      vrni this.m_motorSpeed;
    };
    b2RevoluteJoint.prototype.SetMaxMotorTorque = funkcija (navor) {
      če (navor === nedefiniran) navor = 0;
      this.m_maxMotorTorque = navor;
    };
    b2RevoluteJoint.prototype.GetMotorTorque = funkcija () {
      vrni this.m_maxMotorTorque;
    };
    b2RevoluteJoint.prototype.b2RevoluteJoint = funkcija (def) {
      this.__super.b2Joint.call(this, def);
      this.m_localAnchor1.SetV(def.localAnchorA);
      this.m_localAnchor2.SetV(def.localAnchorB);
      this.m_referenceAngle = def.referenceAngle;
      this.m_impulse.SetZero();
      this.m_motorImpulse = 0,0;
      this.m_lowerAngle = def.lowerAngle;
      this.m_upperAngle = def.upperAngle;
      this.m_maxMotorTorque = def.maxMotorTorque;
      this.m_motorSpeed ​​= def.motorSpeed;
      this.m_enableLimit = def.enableLimit;
      this.m_enableMotor = def.enableMotor;
      this.m_limitState = b2Joint.e_inactiveLimit;
    };
    b2RevoluteJoint.prototype.InitVelocityConstraints = funkcija (korak) {
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var tMat;
      var tX = 0;
      if (this.m_enableMotor || this.m_enableLimit) {
      }
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
      var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
      r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
      var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
      r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
      r2X = tX;
      var m1 = bA.m_invMass;
      var m2 = bB.m_invMass;
      var i1 = bA.m_invI;
      var i2 = bB.m_invI;
      this.m_mass.col1.x = m1 + m2 + r1Y * r1Y * i1 + r2Y * r2Y * i2;
      this.m_mass.col2.x = -r1Y * r1X * i1 - r2Y * r2X * i2;
      this.m_mass.col3.x = -r1Y * i1 - r2Y * i2;
      this.m_mass.col1.y = this.m_mass.col2.x;
      this.m_mass.col2.y = m1 + m2 + r1X * r1X * i1 + r2X * r2X * i2;
      this.m_mass.col3.y = r1X * i1 + r2X * i2;
      this.m_mass.col1.z = this.m_mass.col3.x;
      this.m_mass.col2.z = this.m_mass.col3.y;
      this.m_mass.col3.z = i1 + i2;
      this.m_motorMass = 1,0 / (i1 + i2);
      if (this.m_enableMotor == false) {
        this.m_motorImpulse = 0,0;
      }
      if (this.m_enableLimit) {
        var jointAngle = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
        če (
          b2Math.Abs(this.m_upperAngle - this.m_lowerAngle) <
          2.0 * b2Settings.b2_angularSlop
        ) {
          this.m_limitState = b2Joint.e_equalLimits;
        } else if (jointAngle <= this.m_lowerAngle) {
          if (this.m_limitState != b2Joint.e_atLowerLimit) {
            this.m_impulse.z = 0,0;
          }
          this.m_limitState = b2Joint.e_atLowerLimit;
        } else if (jointAngle >= this.m_upperAngle) {
          if (this.m_limitState != b2Joint.e_atUpperLimit) {
            this.m_impulse.z = 0,0;
          }
          this.m_limitState = b2Joint.e_atUpperLimit;
        } drugače {
          this.m_limitState = b2Joint.e_inactiveLimit;
          this.m_impulse.z = 0,0;
        }
      } drugače {
        this.m_limitState = b2Joint.e_inactiveLimit;
      }
      if (step.warmStarting) {
        this.m_impulse.x *= step.dtRatio;
        this.m_impulse.y *= step.dtRatio;
        this.m_motorImpulse *= step.dtRatio;
        var PX = this.m_impulse.x;
        var PY = this.m_impulse.y;
        bA.m_linearVelocity.x -= m1 * PX;
        bA.m_linearnaHitrost.y -= m1 * PY;
        bA.m_angularVelocity -=
          i1 * (r1X * PY - r1Y * PX + this.m_motorImpulse + this.m_impulse.z);
        bB.m_linearVelocity.x += m2 * PX;
        bB.m_linearnaHitrost.y += m2 * PY;
        bB.m_angularVelocity +=
          i2 * (r2X * PY - r2Y * PX + this.m_motorImpulse + this.m_impulse.z);
      } drugače {
        this.m_impulse.SetZero();
        this.m_motorImpulse = 0,0;
      }
    };
    b2RevoluteJoint.prototype.SolveVelocityConstraints = funkcija (korak) {
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var tMat;
      var tX = 0;
      var newImpulse = 0;
      var r1X = 0;
      var r1Y = 0;
      var r2X = 0;
      var r2Y = 0;
      var v1 = bA.m_linearna hitrost;
      var w1 = bA.m_angularVelocity;
      var v2 = bB.m_linearVelocity;
      var w2 = bB.m_angularVelocity;
      var m1 = bA.m_invMass;
      var m2 = bB.m_invMass;
      var i1 = bA.m_invI;
      var i2 = bB.m_invI;
      if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits) {
        var Cdot = w2 - w1 - this.m_motorSpeed;
        var impulz = this.m_motorMass * -Cdot;
        var oldImpulse = this.m_motorImpulse;
        var maxImpulse = step.dt * this.m_maxMotorTorque;
        this.m_motorImpulse = b2Math.Clamp(
          this.m_motorImpulse + impulz,
          -največji impulz,
          maxImpulse
        );
        impulz = this.m_motorImpulse - stari impulz;
        w1 -= i1 * impulz;
        w2 += i2 * impulz;
      }
      if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
        tMat = bA.m_xf.R;
        r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
        r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
        tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
        r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
        r1X = tX;
        tMat = bB.m_xf.R;
        r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
        r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
        tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
        r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
        r2X = tX;
        var Cdot1X = v2.x + -w2 * r2Y - v1.x - -w1 * r1Y;
        var Cdot1Y = v2.y + w2 * r2X - v1.y - w1 * r1X;
        var Cdot2 = w2 - w1;
        this.m_mass.Solve33(this.impulse3, -Cdot1X, -Cdot1Y, -Cdot2);
        if (this.m_limitState == b2Joint.e_equalLimits) {
          this.m_impulse.Add(this.impulse3);
        } else if (this.m_limitState == b2Joint.e_atLowerLimit) {
          novImpulz = this.m_impulse.z + this.impulse3.z;
          if (newImpulse < 0,0) {
            this.m_mass.Solve22(this.reduced, -Cdot1X, -Cdot1Y);
            this.impulse3.x = this.reduced.x;
            this.impulse3.y = this.reduced.y;
            this.impulse3.z = -this.m_impulse.z;
            this.m_impulse.x += this.reduced.x;
            this.m_impulse.y += this.reduced.y;
            this.m_impulse.z = 0,0;
          }
        } else if (this.m_limitState == b2Joint.e_atUpperLimit) {
          novImpulz = this.m_impulse.z + this.impulse3.z;
          if (newImpulse > 0,0) {
            this.m_mass.Solve22(this.reduced, -Cdot1X, -Cdot1Y);
            this.impulse3.x = this.reduced.x;
            this.impulse3.y = this.reduced.y;
            this.impulse3.z = -this.m_impulse.z;
            this.m_impulse.x += this.reduced.x;
            this.m_impulse.y += this.reduced.y;
            this.m_impulse.z = 0,0;
          }
        }
        v1.x -= m1 * this.impulse3.x;
        v1.y -= m1 * this.impulse3.y;
        w1 -=
          i1 * (r1X * this.impulse3.y - r1Y * this.impulse3.x + this.impulse3.z);
        v2.x += m2 * this.impulse3.x;
        v2.y += m2 * this.impulse3.y;
        w2 +=
          i2 * (r2X * this.impulse3.y - r2Y * this.impulse3.x + this.impulse3.z);
      } drugače {
        tMat = bA.m_xf.R;
        r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
        r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
        tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
        r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
        r1X = tX;
        tMat = bB.m_xf.R;
        r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
        r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
        tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
        r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
        r2X = tX;
        var CdotX = v2.x + -w2 * r2Y - v1.x - -w1 * r1Y;
        var CdotY = v2.y + w2 * r2X - v1.y - w1 * r1X;
        this.m_mass.Solve22(this.impulse2, -CdotX, -CdotY);
        this.m_impulse.x += this.impulse2.x;
        this.m_impulse.y += this.impulse2.y;
        v1.x -= m1 * this.impulse2.x;
        v1.y -= m1 * this.impulse2.y;
        w1 -= i1 * (r1X * this.impulse2.y - r1Y * this.impulse2.x);
        v2.x += m2 * this.impulse2.x;
        v2.y += m2 * ta.impulz2.y;
        w2 += i2 * (r2X * this.impulse2.y - r2Y * this.impulse2.x);
      }
      bA.m_linearnaHitrost.SetV(v1);
      bA.m_angularVelocity = w1;
      bB.m_linearnaHitrost.SetV(v2);
      bB.m_angularVelocity = w2;
    };
    b2RevoluteJoint.prototype.SolvePositionConstraints = funkcija (baumgarte) {
      if (baumgarte === nedefinirano) baumgarte = 0;
      var oldLimitImpulse = 0;
      var C = 0;
      var tMat;
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var angularError = 0,0;
      var positionError = 0,0;
      var tX = 0;
      var impulzX = 0;
      var impulzY = 0;
      if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
        var angle = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
        var limitImpulse = 0,0;
        if (this.m_limitState == b2Joint.e_equalLimits) {
          C = b2Math.Clamp(
            kot - this.m_lowerAngle,
            -b2Settings.b2_maxAngularCorrection,
            b2Settings.b2_maxAngularCorrection
          );
          limitImpulse = -this.m_motorMass * C;
          angularError = b2Math.Abs(C);
        } else if (this.m_limitState == b2Joint.e_atLowerLimit) {
          C = kot - this.m_lowerAngle;
          angularError = -C;
          C = b2Math.Clamp(
            C + b2Settings.b2_angularSlop,
            -b2Settings.b2_maxAngularCorrection,
            0,0
          );
          limitImpulse = -this.m_motorMass * C;
        } else if (this.m_limitState == b2Joint.e_atUpperLimit) {
          C = kot - this.m_upperAngle;
          angularError = C;
          C = b2Math.Clamp(
            C - b2Settings.b2_angularSlop,
            0,0,
            b2Settings.b2_maxAngularCorrection
          );
          limitImpulse = -this.m_motorMass * C;
        }
        bA.m_sweep.a -= bA.m_invI * limitImpulse;
        bB.m_sweep.a += bB.m_invI * limitImpulse;
        bA.SynchronizeTransform();
        bB.SynchronizeTransform();
      }
      {
        tMat = bA.m_xf.R;
        var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
        var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
        tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
        r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
        r1X = tX;
        tMat = bB.m_xf.R;
        var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
        var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
        tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
        r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
        r2X = tX;
        var CX = bB.m_sweep.cx + r2X - bA.m_sweep.cx - r1X;
        var CY = bB.m_sweep.cy + r2Y - bA.m_sweep.cy - r1Y;
        var CLengthSquared = CX * CX + CY * CY;
        var CLength = Math.sqrt(CLengthSquared);
        positionError = CLength;
        var invMass1 = bA.m_invMass;
        var invMass2 = bB.m_invMass;
        var invI1 = bA.m_invI;
        var invI2 = bB.m_invI;
        var k_allowedStretch = 10,0 * b2Settings.b2_linearSlop;
        if (CLengthSquared > k_allowedStretch * k_allowedStretch) {
          var uX = CX / CLength;
          var uY = CY / CLength;
          var k = invMass1 + invMass2;
          var m = 1,0 / k;
          impulzX = m * -CX;
          impulzY = m * -CY;
          var k_beta = 0,5;
          bA.m_sweep.cx -= k_beta * invMass1 * impulzX;
          bA.m_sweep.cy -= k_beta * invMass1 * impulzY;
          bB.m_sweep.cx += k_beta * invMass2 * impulzX;
          bB.m_sweep.cy += k_beta * invMass2 * impulzY;
          CX = bB.m_sweep.cx + r2X - bA.m_sweep.cx - r1X;
          CY = bB.m_sweep.cy + r2Y - bA.m_sweep.cy - r1Y;
        }
        this.K1.col1.x = invMass1 + invMass2;
        this.K1.col2.x = 0,0;
        this.K1.col1.y = 0,0;
        this.K1.col2.y = invMass1 + invMass2;
        this.K2.col1.x = invI1 * r1Y * r1Y;
        this.K2.col2.x = -invI1 * r1X * r1Y;
        this.K2.col1.y = -invI1 * r1X * r1Y;
        this.K2.col2.y = invI1 * r1X * r1X;
        this.K3.col1.x = invI2 * r2Y * r2Y;
        this.K3.col2.x = -invI2 * r2X * r2Y;
        this.K3.col1.y = -invI2 * r2X * r2Y;
        this.K3.col2.y = invI2 * r2X * r2X;
        this.K.SetM(this.K1);
        to.K.DodajM(ta.K2);
        to.K.DodajM(ta.K3);
        this.K.Solve(b2RevoluteJoint.tImpulse, -CX, -CY);
        impulzX = b2RevoluteJoint.tImpulse.x;
        impulzY = b2RevoluteJoint.tImpulse.y;
        bA.m_sweep.cx -= bA.m_invMass * impulzX;
        bA.m_sweep.cy -= bA.m_invMass * impulzY;
        bA.m_sweep.a -= bA.m_invI * (r1X * impulzY - r1Y * impulzX);
        bB.m_sweep.cx += bB.m_invMass * impulzX;
        bB.m_sweep.cy += bB.m_invMass * impulzY;
        bB.m_sweep.a += bB.m_invI * (r2X * impulzY - r2Y * impulzX);
        bA.SynchronizeTransform();
        bB.SynchronizeTransform();
      }
      vrnitev (
        positionError <= b2Settings.b2_linearSlop &&
        angularError <= b2Settings.b2_angularSlop
      );
    };
    Box2D.postDefs.push(funkcija () {
      Box2D.Dynamics.Joints.b2RevoluteJoint.tImpulse = novo b2Vec2();
    });
    Box2D.inherit(b2RevoluteJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2RevoluteJointDef.prototype.__super =
      Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2RevoluteJointDef.b2RevoluteJointDef = funkcija () {
      Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(to, argumenti);
      this.localAnchorA = novo b2Vec2();
      this.localAnchorB = novo b2Vec2();
    };
    b2RevoluteJointDef.prototype.b2RevoluteJointDef = funkcija () {
      this.__super.b2JointDef.call(this);
      this.type = b2Joint.e_revoluteJoint;
      this.localAnchorA.Set(0,0, 0,0);
      this.localAnchorB.Set(0,0, 0,0);
      this.referenceAngle = 0,0;
      this.lowerAngle = 0,0;
      this.upperAngle = 0,0;
      this.maxMotorTorque = 0,0;
      this.motorSpeed ​​= 0,0;
      this.enableLimit = false;
      this.enableMotor = false;
    };
    b2RevoluteJointDef.prototype.Initialize = funkcija (bA, bB, sidro) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA = this.bodyA.GetLocalPoint(sidro);
      this.localAnchorB = this.bodyB.GetLocalPoint(sidro);
      this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    };
    Box2D.inherit(b2WeldJoint, Box2D.Dynamics.Joints.b2Joint);
    b2WeldJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2WeldJoint.b2WeldJoint = funkcija () {
      Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
      this.m_localAnchorA = novo b2Vec2();
      this.m_localAnchorB = novo b2Vec2();
      this.m_impulse = novo b2Vec3();
      this.m_mass = novo b2Mat33();
    };
    b2WeldJoint.prototype.GetAnchorA = funkcija () {
      vrni this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
    };
    b2WeldJoint.prototype.GetAnchorB = funkcija () {
      vrni this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
    };
    b2WeldJoint.prototype.GetReactionForce = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      vrni novo b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
    };
    b2WeldJoint.prototype.GetReactionTorque = funkcija (inv_dt) {
      if (inv_dt === nedefinirano) inv_dt = 0;
      return inv_dt * this.m_impulse.z;
    };
    b2WeldJoint.prototype.b2WeldJoint = funkcija (def) {
      this.__super.b2Joint.call(this, def);
      this.m_localAnchorA.SetV(def.localAnchorA);
      this.m_localAnchorB.SetV(def.localAnchorB);
      this.m_referenceAngle = def.referenceAngle;
      this.m_impulse.SetZero();
      this.m_mass = novo b2Mat33();
    };
    b2WeldJoint.prototype.InitVelocityConstraints = funkcija (korak) {
      var tMat;
      var tX = 0;
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      tMat = bA.m_xf.R;
      var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
      var rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
      tX = tMat.col1.x * rAX + tMat.col2.x * rAY;
      rAY = tMat.col1.y * rAX + tMat.col2.y * rAY;
      rAX = tX;
      tMat = bB.m_xf.R;
      var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
      var rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * rBX + tMat.col2.x * rBY;
      rBY = tMat.col1.y * rBX + tMat.col2.y * rBY;
      rBX = tX;
      var mA = bA.m_invMass;
      var mB = bB.m_invMass;
      var iA = bA.m_invI;
      var iB = bB.m_invI;
      this.m_mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB;
      this.m_mass.col2.x = -rAY * rAX * iA - rBY * rBX * iB;
      this.m_mass.col3.x = -rAY * iA - rBY * iB;
      this.m_mass.col1.y = this.m_mass.col2.x;
      this.m_mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB;
      this.m_mass.col3.y = rAX * iA + rBX * iB;
      this.m_mass.col1.z = this.m_mass.col3.x;
      this.m_mass.col2.z = this.m_mass.col3.y;
      this.m_mass.col3.z = iA + iB;
      if (step.warmStarting) {
        this.m_impulse.x *= step.dtRatio;
        this.m_impulse.y *= step.dtRatio;
        this.m_impulse.z *= step.dtRatio;
        bA.m_linearVelocity.x -= mA * this.m_impulse.x;
        bA.m_linearnaHitrost.y -= mA * this.m_impulz.y;
        bA.m_angularVelocity -=
          iA *
          (rAX * this.m_impulse.y - rAY * this.m_impulse.x + this.m_impulse.z);
        bB.m_linearVelocity.x += mB * this.m_impulse.x;
        bB.m_linearVelocity.y += mB * this.m_impulse.y;
        bB.m_angularVelocity +=
          iB *
          (rBX * this.m_impulse.y - rBY * this.m_impulse.x + this.m_impulse.z);
      } drugače {
        this.m_impulse.SetZero();
      }
    };
    b2WeldJoint.prototype.SolveVelocityConstraints = funkcija (korak) {
      var tMat;
      var tX = 0;
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      var vA = bA.m_linearna hitrost;
      var wA = bA.m_angularVelocity;
      var vB = bB.m_linearnaHitrost;
      var wB = bB.m_angularVelocity;
      var mA = bA.m_invMass;
      var mB = bB.m_invMass;
      var iA = bA.m_invI;
      var iB = bB.m_invI;
      tMat = bA.m_xf.R;
      var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
      var rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
      tX = tMat.col1.x * rAX + tMat.col2.x * rAY;
      rAY = tMat.col1.y * rAX + tMat.col2.y * rAY;
      rAX = tX;
      tMat = bB.m_xf.R;
      var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
      var rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * rBX + tMat.col2.x * rBY;
      rBY = tMat.col1.y * rBX + tMat.col2.y * rBY;
      rBX = tX;
      var Cdot1X = vB.x - wB * rBY - vA.x + wA * rAY;
      var Cdot1Y = vB.y + wB * rBX - vA.y - wA * rAX;
      var Cdot2 = wB - wA;
      var impulz = novo b2Vec3();
      this.m_mass.Solve33(impulse, -Cdot1X, -Cdot1Y, -Cdot2);
      this.m_impulse.Add(impulse);
      vA.x -= mA * impulz.x;
      vA.y -= mA * impulz.y;
      wA -= iA * (rAX * impulz.y - rAY * impulz.x + impulz.z);
      vB.x += mB * impulz.x;
      vB.y += mB * impulz.y;
      wB += iB * (rBX * impulz.y - rBY * impulz.x + impulz.z);
      bA.m_angularVelocity = wA;
      bB.m_angularVelocity = wB;
    };
    b2WeldJoint.prototype.SolvePositionConstraints = funkcija (baumgarte) {
      if (baumgarte === nedefinirano) baumgarte = 0;
      var tMat;
      var tX = 0;
      var bA = this.m_bodyA;
      var bB = this.m_bodyB;
      tMat = bA.m_xf.R;
      var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
      var rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
      tX = tMat.col1.x * rAX + tMat.col2.x * rAY;
      rAY = tMat.col1.y * rAX + tMat.col2.y * rAY;
      rAX = tX;
      tMat = bB.m_xf.R;
      var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
      var rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
      tX = tMat.col1.x * rBX + tMat.col2.x * rBY;
      rBY = tMat.col1.y * rBX + tMat.col2.y * rBY;
      rBX = tX;
      var mA = bA.m_invMass;
      var mB = bB.m_invMass;
      var iA = bA.m_invI;
      var iB = bB.m_invI;
      var C1X = bB.m_sweep.cx + rBX - bA.m_sweep.cx - rAX;
      var C1Y = bB.m_sweep.cy + rBY - bA.m_sweep.cy - rAY;
      var C2 = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
      var k_allowedStretch = 10,0 * b2Settings.b2_linearSlop;
      var positionError = Math.sqrt(C1X * C1X + C1Y * C1Y);
      var angularError = b2Math.Abs(C2);
      if (positionError > k_allowedStretch) {
        iA *= 1,0;
        iB *= 1,0;
      }
      this.m_mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB;
      this.m_mass.col2.x = -rAY * rAX * iA - rBY * rBX * iB;
      this.m_mass.col3.x = -rAY * iA - rBY * iB;
      this.m_mass.col1.y = this.m_mass.col2.x;
      this.m_mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB;
      this.m_mass.col3.y = rAX * iA + rBX * iB;
      this.m_mass.col1.z = this.m_mass.col3.x;
      this.m_mass.col2.z = this.m_mass.col3.y;
      this.m_mass.col3.z = iA + iB;
      var impulz = novo b2Vec3();
      this.m_mass.Solve33(impulz, -C1X, -C1Y, -C2);
      bA.m_sweep.cx -= mA * impulz.x;
      bA.m_sweep.cy -= mA * impulz.y;
      bA.m_sweep.a -= iA * (rAX * impulz.y - rAY * impulz.x + impulz.z);
      bB.m_sweep.cx += mB * impulz.x;
      bB.m_sweep.cy += mB * impulz.y;
      bB.m_sweep.a += iB * (rBX * impulz.y - rBY * impulz.x + impulz.z);
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
      vrnitev (
        positionError <= b2Settings.b2_linearSlop &&
        angularError <= b2Settings.b2_angularSlop
      );
    };
    Box2D.inherit(b2WeldJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2WeldJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2WeldJointDef.b2WeldJointDef = funkcija () {
      Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(to, argumenti);
      this.localAnchorA = novo b2Vec2();
      this.localAnchorB = novo b2Vec2();
    };
    b2WeldJointDef.prototype.b2WeldJointDef = funkcija () {
      this.__super.b2JointDef.call(this);
      this.type = b2Joint.e_weldJoint;
      this.referenceAngle = 0,0;
    };
    b2WeldJointDef.prototype.Initialize = funkcija (bA, bB, sidro) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA.SetV(this.bodyA.GetLocalPoint(sidro));
      this.localAnchorB.SetV(this.bodyB.GetLocalPoint(sidro));
      this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    };
  })();
  (funkcija () {
    var b2DebugDraw = Box2D.Dynamics.b2DebugDraw;
    b2DebugDraw.b2DebugDraw = funkcija () {
      this.m_drawScale = 1,0;
      this.m_lineThickness = 1,0;
      this.m_alpha = 1,0;
      this.m_fillAlpha = 1,0;
      this.m_xformScale = 1,0;
      var __this = to;
      //#ZAOBODNA REŠITEV
      this.m_sprite = {
        grafika: {
          jasno: funkcija () {
            __this.m_ctx.clearRect(
              0,
              0,
              __this.m_ctx.canvas.width,
              __this.m_ctx.canvas.height
            );
          },
        },
      };
    };
    b2DebugDraw.prototype._color = funkcija (barva, alfa) {
      vrnitev (
        "rgba(" +
        ((barva & 0xff0000) >> 16) +
        "," +
        ((barva & 0xff00) >> 8) +
        "," +
        (barva & 0xff) +
        "," +
        alfa +
        ")"
      );
    };
    b2DebugDraw.prototype.b2DebugDraw = funkcija () {
      this.m_drawFlags = 0;
    };
    b2DebugDraw.prototype.SetFlags = funkcija (zastavice) {
      če (zastavice === nedefinirano) zastavice = 0;
      this.m_drawFlags = zastavice;
    };
    b2DebugDraw.prototype.GetFlags = funkcija () {
      vrni this.m_drawFlags;
    };
    b2DebugDraw.prototype.AppendFlags = funkcija (zastavice) {
      če (zastavice === nedefinirano) zastavice = 0;
      this.m_drawFlags |= zastavice;
    };
    b2DebugDraw.prototype.ClearFlags = funkcija (zastavice) {
      če (zastavice === nedefinirano) zastavice = 0;
      this.m_drawFlags &= ~flags;
    };
    b2DebugDraw.prototype.SetSprite = funkcija (sprite) {
      this.m_ctx = sprite;
    };
    b2DebugDraw.prototype.GetSprite = funkcija () {
      vrni this.m_ctx;
    };
    b2DebugDraw.prototype.SetDrawScale = funkcija (drawScale) {
      if (drawScale === nedefinirano) drawScale = 0;
      this.m_drawScale = drawScale;
    };
    b2DebugDraw.prototype.GetDrawScale = funkcija () {
      vrni this.m_drawScale;
    };
    b2DebugDraw.prototype.SetLineThickness = funkcija (lineThickness) {
      if (Debelina črte === nedefinirano) Debelina črte = 0;
      this.m_lineThickness = lineThickness;
      this.m_ctx.strokeWidth = lineThickness;
    };
    b2DebugDraw.prototype.GetLineThickness = funkcija () {
      vrni this.m_lineThickness;
    };
    b2DebugDraw.prototype.SetAlpha = funkcija (alfa) {
      če (alfa === nedefinirano) alfa = 0;
      this.m_alpha = alfa;
    };
    b2DebugDraw.prototype.GetAlpha = funkcija () {
      vrni to.m_alpha;
    };
    b2DebugDraw.prototype.SetFillAlpha = funkcija (alfa) {
      če (alfa === nedefinirano) alfa = 0;
      this.m_fillAlpha = alfa;
    };
    b2DebugDraw.prototype.GetFillAlpha = funkcija () {
      vrni this.m_fillAlpha;
    };
    b2DebugDraw.prototype.SetXFormScale = funkcija (xformScale) {
      if (xformScale === nedefinirano) xformScale = 0;
      this.m_xformScale = xformScale;
    };
    b2DebugDraw.prototype.GetXFormScale = funkcija () {
      vrni this.m_xformScale;
    };
    b2DebugDraw.prototype.DrawPolygon = funkcija (točke, vertexCount, barva) {
      if (!vertexCount) return;
      var s = this.m_ctx;
      var drawScale = this.m_drawScale;
      s.beginPath();
      s.strokeStyle = this._color(color.color, this.m_alpha);
      s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
      for (var i = 1; i < vertexCount; i++) {
        s.lineTo(vozlišča[i].x * drawScale, vozlišča[i].y * drawScale);
      }
      s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
      s.closePath();
      s.stroke();
    };
    b2DebugDraw.prototype.DrawSolidPolygon = funkcija (
      oglišča,
      vertexCount,
      barva
    ) {
      if (!vertexCount) return;
      var s = this.m_ctx;
      var drawScale = this.m_drawScale;
      s.beginPath();
      s.strokeStyle = this._color(color.color, this.m_alpha);
      s.fillStyle = this._color(color.color, this.m_fillAlpha);
      s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
      for (var i = 1; i < vertexCount; i++) {
        s.lineTo(vozlišča[i].x * drawScale, vozlišča[i].y * drawScale);
      }
      s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
      s.closePath();
      s.fill();
      s.stroke();
    };
    b2DebugDraw.prototype.DrawCircle = funkcija (središče, polmer, barva) {
      if (!radius) return;
      var s = this.m_ctx;
      var drawScale = this.m_drawScale;
      s.beginPath();
      s.strokeStyle = this._color(color.color, this.m_alpha);
      s.arc(
        center.x * drawScale,
        center.y * drawScale,
        polmer * drawScale,
        0,
        Math.PI * 2,
        prav
      );
      s.closePath();
      s.stroke();
    };
    b2DebugDraw.prototype.DrawSolidCircle = funkcija (
      center,
      polmer,
      os,
      barva
    ) {
      if (!radius) return;
      var s = this.m_ctx,
        drawScale = this.m_drawScale,
        cx = center.x * drawScale,
        cy = center.y * drawScale;
      s.premakni(0, 0);
      s.beginPath();
      s.strokeStyle = this._color(color.color, this.m_alpha);
      s.fillStyle = this._color(color.color, this.m_fillAlpha);
      s.arc(cx, cy, radius * drawScale, 0, Math.PI * 2, true);
      s.premakni(cx, cy);
      s.lineTo(
        (center.x + axis.x * polmer) * drawScale,
        (center.y + os.y * polmer) * drawScale
      );
      s.closePath();
      s.fill();
      s.stroke();
    };
    b2DebugDraw.prototype.DrawSegment = funkcija (p1, p2, barva) {
      var s = this.m_ctx,
        drawScale = this.m_drawScale;
      s.strokeStyle = this._color(color.color, this.m_alpha);
      s.beginPath();
      s.moveTo(p1.x * drawScale, p1.y * drawScale);
      s.lineTo(p2.x * drawScale, p2.y * drawScale);
      s.closePath();
      s.stroke();
    };
    b2DebugDraw.prototype.DrawTransform = funkcija (xf) {
      var s = this.m_ctx,
        drawScale = this.m_drawScale;
      s.beginPath();
      s.strokeStyle = this._color(0xff0000, this.m_alpha);
      s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
      s.lineTo(
        (xf.position.x + this.m_xformScale * xf.R.col1.x) * drawScale,
        (xf.position.y + this.m_xformScale * xf.R.col1.y) * drawScale
      );

      s.strokeStyle = this._color(0xff00, this.m_alpha);
      s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
      s.lineTo(
        (xf.position.x + this.m_xformScale * xf.R.col2.x) * drawScale,
        (xf.position.y + this.m_xformScale * xf.R.col2.y) * drawScale
      );
      s.closePath();
      s.stroke();
    };
  })();
  var i;
  for (i = 0; i < Box2D.postDefs.length; ++i) Box2D.postDefs[i]();

  const ArgumentType = Scratch.ArgumentType;
  const BlockType = Scratch.BlockType;
  // const MathUtil = require('../../util/math-util');
  // const Clone = require('../../util/clone');
  const Cast = {
    doŠtevilka: (n) => +n || 0,
  };
  // const Cast = require('../../util/cast');
  // const Runtime = require('../../engine/runtime');
  // const RenderedTarget = require('../../sprites/rendered-target');
  // const formatMessage = require('format-message');
  const formatMessage = (obj) => obj.default;
  // const MathUtil = require('../../util/math-util');
  // const Timer = require('../../util/timer');
  // const Matter = require('matterJs/matter');
  // const Matter = require('matter-js');
  const ROTATION_STYLE_ALL_AROUND = "povsod naokrog";

  // const Box2D = require('./Box2d.min').box2d;

  // window.decomp = require('poly-decomp');

  const b2World = Box2D.Dynamics.b2World;
  const b2Vec2 = Box2D.Common.Math.b2Vec2;
  const b2AABB = Box2D.Collision.b2AABB;
  const b2BodyDef = Box2D.Dynamics.b2BodyDef;
  const b2Body = Box2D.Dynamics.b2Body;
  const b2FixtureDef = Box2D.Dynamics.b2FixtureDef;
  // const b2Fixture = Box2D.Dynamics.b2Fixture;
  // const b2Fixture = Box2D.Dynamics.b2Fixture;
  const b2Contact = Box2D.Dynamics.Contacts.b2Contact;
  // const b2MassData = Box2D.Collision.Shapes.b2MassData;
  const b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;
  const b2CircleShape = Box2D.Collision.Shapes.b2CircleShape;
  // const b2DebugDraw = Box2D.Dynamics.b2DebugDraw;
  const b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef;
  const b2Math = Box2D.Common.Math.b2Math;

  naj svet;
  pusti povečavo;

  const fixDef = novo b2FixtureDef();
  const bodyDef = novo b2BodyDef();

  // const uid_seq = 0;
  // naj bo ujidSeq = 0;

  const prevPos = {};
  /**
  * Aktivni b2Body/s na svetu.
  * @type {Object.<niz,*>}
  */
  const telesa = {};
  // const spoji = {};
  const pripet = {}; // Zemljevid ID-jev za zatične sklepe
  /**
  * Izvajalno okolje, ki instancira ta paket blokov.
  * @type {Array}
  */
  const stageBodies = [];

  // const categorySeq = 1;
  // kategorije const = {privzeto: 1};

  const bodyCategoryBits = 1;
  const bodyMaskBits = 1;
  // const noCollideSeq = 0;

  const toRad = Math.PI / 180;

  // Uporablja se za beleženje položaja drsenja vseh spritejev
  const _scroll = novo b2Vec2(0, 0);

  const STAGE_TYPE_OPTIONS = {
    BOXED: "zapakirano",
    FLOOR: "tla",
    OPEN: "odprto",
  };

  const SPACE_TYPE_OPTIONS = {
    SVET: "svet",
    STAGE: "oder",
    RELATIVNO: "sorodnik",
  };

  const WHERE_TYPE_OPTIONS = {
    ANY: "kateri koli",
    NOGE: "noge",
  };

  const SHAPE_TYPE_OPTIONS = {
    KOSTUM: "kostum",
    CIRCLE: "krog",
    SVG_POLYGON: "svg",
    VSI: "vse",
  };

  const _definePolyFromHull = funkcija (hullPoints) {
    fixDef.shape = new b2PolygonShape();

    const vertices = [];

    let prev = null;
    for (naj i = hullPoints.length - 1; i >= 0; i--) {
      // za (naj bo i = 0; i < hullPoints.length; i++) {
      const b2Vec = novo b2Vec2(hullPoints[i].x / povečava, hullPoints[i].y / povečava);
      če (
        prejšnji !== null &&
        b2Math.SubtractVV(b2Vec, prev).LengthSquared() > Number.MIN_VALUE
      ) {
        vertices.push(b2Vec);
      }
      prejšnji = b2Vec;
    }

    fixDef.shape.SetAsArray(vozlišča);
  };

  const _placeBody = funkcija (id, x, y, dir) {
    if (telesa[id]) {
      world.DestroyBody(telesa[id]);
    }

    fixDef.filter.categoryBits = bodyCategoryBits;
    fixDef.filter.maskBits = bodyMaskBits;

    bodyDef.position.x = (x + _scroll.x) / povečava;
    bodyDef.position.y = (y + _scroll.y) / povečava;
    bodyDef.angle = (90 - dir) * toRad;

    const body = world.CreateBody(bodyDef);
    body.uid = id;
    body.CreateFixture(fixDef);
    telesa [id] = telo;
    povratno telo;
  };

  const _applyForce = funkcija (id, ftype, x, y, dir, pow) {
    const telo = telesa[id];
    if (!body) {
      vrnitev;
    }

    dir = (90 - dir) * toRad;

    if (ftype === "Impulz") {
      const center = body.GetLocalCenter(); // pridobi podatke o masi iz svojega telesa

      body.ApplyImpulse(
        {x: pow * Math.cos(dir), y: pow * Math.sin(dir)},
        body.GetWorldPoint({ x: x / povečava + center.x, y: y / povečava + center.y })
      );
    } else if (ftype === "World Impulse") {
      body.ApplyForce(
        {x: pow * Math.cos(dir), y: pow * Math.sin(dir)},
        { x: x / povečava, y: y / povečava }
      );
    }
  };

  // ['', 'Določi dolžino vzmeti: %n Dušenje: %n Freq: %n', '_defineSpring', 100, 0,5, 8],
  const defSpring = {len: 100, vlaga: 0,7, frekvenca: 5};
  const _defineSpring = funkcija (len, damp, freq) {
    defSpring.len = len < 0,1? 0,1 : len / povečava;
    defSpring.damp = vlažno < 0? 0,7 : vlažno;
    defSpring.freq = freq > 0? frekvenca: 5;
  };

  const _createJointOfType = funkcija (
    jName,
    tip,
    bodyID,
    x,
    y,
    bodyID2,
    x2,
    y2
  ) {
    // če (jName.length > 0) ext.destroyJoint(jName);

    if (!bodyID) bodyID = null;
    if (!bodyID2) bodyID2 = nič;
    if (!bodyID && !bodyID2) {
      vrni nič;
    }

    const body = bodyID? telesa[bodyID] : world.GetGroundBody();
    const body2 = bodyID2? telesa[bodyID2] : world.GetGroundBody();

    if (!body || !body2) vrne nič;

    naj md;
    stikalo (tip) {
      primer "pomlad":
        md = novo Box2D.Dynamics.Joints.b2DistanceJointDef();
        md.length = defSpring.len;
        md.dampingRatio = defSpring.damp;
        md.frequencyHz = defSpring.freq;
        md.bodyA = telo;
        md.bodyB = telo2;
        md.localAnchorA = { x: x / povečava, y: y / povečava };
        md.localAnchorB = { x: x2 / povečava, y: y2 / povečava };
        odmor;

      primer "Vrtljiv":
        md = novo Box2D.Dynamics.Joints.b2RevoluteJointDef();
        md.bodyA = telo;
        md.bodyB = telo2;
        md.localAnchorA = { x: x / povečava, y: y / povečava };
        če (x2 === nič) {
          če (telo2) {
            md.localAnchorB = body2.GetLocalPoint(body.GetPosition()); // Kolesni spoj ...
          } drugače {
            md.localAnchorB = body.GetWorldPoint({ x: x / povečava, y: y / povečava });
          }
        } drugače {
          md.localAnchorB = { x: x2 / povečava, y: y2 / povečava };
        }
        odmor;

      primer "miška":
        md = novo b2MouseJointDef();
        if (bodyID) {
          md.bodyB = telo;
          md.target.Set(x / povečava, y / povečava);
        } drugače {
          md.bodyB = telo2;
          md.target.Set(x2 / povečava, y2 / povečava);
        }
        md.bodyA = world.GetGroundBody();
        md.collideConnected = res;
        md.maxForce = 300,0 * body.GetMass();
        odmor;
    }

    // md.collideConnected = res;
    // md.maxForce = 300,0 * body.GetMass();
    const joint = world.CreateJoint(md);
    if (bodyID) {
      body.SetAwake(true);
    }
    if (bodyID2) {
      body2.SetAwake(true);
    }

    // if (!jName) {
    // ujidSeq++;
    // jName = `_${ujidSeq}`;
    // }
    // spoji [jName] = spoj;
    povratni spoj;
  };

  /**
  * Nastavite koordinate X in Y (brez ograje)
  * @param {!RenderedTarget} rt renderedTarget.
  * @param {!number} x Nova koordinata X, v koordinatah Scratch.
  * @param {!number} y Nova koordinata Y, v koordinatah Scratch.
  * @param {?boolean} force Force nastavitev X/Y, v primeru vlečenja
  */
  const _setXY = funkcija (rt, x, y, sila) {
    if (rt.isStage) return;
    if (rt.dragging && !force) return;
    const oldX = rt.x;
    const oldY = rt.y;
    if (rt.renderer) {
      // const position = rt.renderer.getFencedPositionOfDrawable(rt.drawableID, [x, y]);
      rt.x = x; // položaj [0];
      rt.y = y; // položaj [1];

      rt.renderer.updateDrawableProperties(rt.drawableID, {
        položaj: [x, y],
      });
      if (rt.visible) {
        rt.emit("CILJ_DOGODKA_VIZUALNA_SPREMEMBA", rt);
        rt.runtime.requestRedraw();
      }
    } drugače {
      rt.x = x;
      rt.y = y;
    }
    rt.emit("CILJ_DOGODKA_PREMAKNJEN", rt, oldX, oldY, sila);
    rt.runtime.requestTargetsUpdate(rt);
  };

  const createStageBody = funkcija () {
    const body = world.CreateBody(bodyDef);
    body.CreateFixture(fixDef);
    stageBodies.push(telo);
  };

  const _setStageType = funkcija (tip) {
    // Počisti prejšnjo stopnjo
    if (stageBodies.length > 0) {
      for (const stageBodyID v stageBodies) {
        world.DestroyBody(stageBodies[stageBodyID]);
        izbriši stageBodies[stageBodyID];
      }
    }

    // Zgradite novo stopnjo
    bodyDef.type = b2Body.b2_staticBody;
    fixDef.shape = new b2PolygonShape();
    bodyDef.angle = 0;

    če (tip === STAGE_TYPE_OPTIONS.BOXED) {
      fixDef.shape.SetAsBox(250 / povečava, 10 / povečava);
      bodyDef.position.Set(0, -190 / povečava);
      createStageBody();
      bodyDef.position.Set(0, 1000 / povečava);
      createStageBody();
      fixDef.shape.SetAsBox(10 / povečava, 800 / povečava);
      bodyDef.position.Set(-250 / povečava, 540 / povečava);
      createStageBody();
      bodyDef.position.Set(250 / povečava, 540 / povečava);
      createStageBody();
    } sicer če (tip === STAGE_TYPE_OPTIONS.FLOOR) {
      fixDef.shape.SetAsBox(5000 / povečava, 100 / povečava);
      bodyDef.position.Set(0, -280 / povečava);
      createStageBody();
      bodyDef.position.Set(-10000, -280 / povečava);
      createStageBody();
      bodyDef.position.Set(10000, -280 / povečava);
      createStageBody();
      bodyDef.position.Set(-20000, -280 / povečava);
      createStageBody();
      bodyDef.position.Set(20000, -280 / povečava);
      createStageBody();
    }

    bodyDef.type = b2Body.b2_dynamicBody;

    for (const bodyID v telesih) {
      telesa [ID telesa]. SetAwake (true);
    }
  };

  const blockIconURI = "data:image/svg+xml;base64,+DQo8cmVjdCB4PSIxLjUiIHk9IjE2LjMiIGZpbGw9IiNGRkZGRkYiIHN0cm9rZT0iIzE2OUZCMCIgc3Ryb2tlLXdpZHRoPSIzIiB3aWR0aD0iMTQuOCIgaGVpZ2h0PSIxNC44Ii8+DQo8cmVjdCB4PSIxNi4zIiB5PSIxNi4zIiBmaWxsPSIjRkZGRkZGIiBzdHJva2U9IiMxNjlGQjAiIHN0cm9rZS13aWR0aD0iMyIgd2lkdGg9IjE0LjgiIGhlaWdodD0iMTQuOCIvPg0KPC9zdmc+";
  const menuIconURI = "data:image/svg+xml;base64,+DQo8cmVjdCB4PSIxLjUiIHk9IjE2LjMiIGZpbGw9IiNGRkZGRkYiIHN0cm9rZT0iIzE2OUZCMCIgc3Ryb2tlLXdpZHRoPSIzIiB3aWR0aD0iMTQuOCIgaGVpZ2h0PSIxNC44Ii8+DQo8cmVjdCB4PSIxNi4zIiB5PSIxNi4zIiBmaWxsPSIjRkZGRkZGIiBzdHJva2U9IiMxNjlGQjAiIHN0cm9rZS13aWR0aD0iMyIgd2lkdGg9IjE0LjgiIGhlaWdodD0iMTQuOCIvPg0KPC9zdmc+";
  const vm = Scratch.vm;

  razred Scratch3Griffpatch {
    konstruktor() {
      /**
       * Izvajalno okolje, ki instancira ta paket blokov.
       * @type {Runtime}
       */
      this.runtime = vm.runtime;

      // Počisti ciljne vrednosti stanja gibanja, ko se projekt začne.
      this.runtime.on("PROJECT_START", this.reset.bind(this));

      svet = nov b2Svet(
        novo b2Vec2(0, -10), // gravitacija (10)
        res // dovoli spanje
      );

      povečava = 50; // lestvica;

      this.map = {};

      fixDef.density = 1,0; // 1.0
      fixDef.friction = 0,5; // 0,5
      fixDef.restitution = 0,2; // 0,2

      _setStageType(STAGE_TYPE_OPTIONS.BOXED);
    }

    ponastaviti() {
      za (const telo v telesih) {
        if (pripeto[body.uid]) {
          world.DestroyJoint(pinned[body.uid]);
          izbriši pripeto [body.uid];
        }
        world.DestroyBody(telesa[telo]);
        izbriši telesa [telo];
        izbriši prevPos[telo];
      }
      // todo: izbrisati spoje?
    }

    /**
     * Ključ za nalaganje in shranjevanje ciljnega stanja, povezanega z glasbo.
     * @type {niz}
     */
    static get STATE_KEY() {
      vrni "Scratch.Griffpatch";
    }

    /**
     * @vrne metapodatke {object} za to razširitev in njene bloke.
     */
    getInfo() {
      return {
        id: "griffpatch",
        ime: formatMessage({
          id: "griffpatch.categoryName",
          privzeto: "Fizika",
          opis: "Oznaka za kategorijo razširitve Griffpatch",
        }),
        menuIconURI: menuIconURI,
        blockIconURI: blockIconURI,
        bloki: [
          // Globalna nastavitev ------------------

          {
            opcode: "setStage",
            blockType: BlockType.COMMAND,
            besedilo: formatMessage({
              id: "griffpatch.setStage",
              privzeto: "stopnja nastavitve [stageType]",
              opis: "Nastavi vrsto stopnje",
            }),
            argumenti: {
              stageType: {
                tip: ArgumentType.STRING,
                meni: "StageTypes",
                privzeta vrednost: STAGE_TYPE_OPTIONS.BOXED,
              },
            },
          },
          {
            opcode: "setGravity",
            blockType: BlockType.COMMAND,
            besedilo: formatMessage({
              id: "griffpatch.setGravity",
              privzeto: "nastavi gravitacijo na x: [gx] y: [gy]",
              opis: "Nastavi gravitacijo",
            }),
            argumenti: {
              gx: {
                tip: ArgumentType.NUMBER,
                privzeta vrednost: 0,
              },
              gy: {
                tip: ArgumentType.NUMBER,
                privzeta vrednost: -10,
              },
            },
          },

          "---",

          {
            opcode: "setPhysics",
            blockType: BlockType.COMMAND,
            besedilo: formatMessage({
              id: "griffpatch.setPhysics",
              privzeto: "omogoči za način [oblika] [način]",
              opis: "Omogoči fiziko za ta duh",
            }),
            argumenti: {
              oblika: {
                tip: ArgumentType.STRING,
                meni: "Vrste oblik",
                defaultValue: "kostum",
              },
              način: {
                tip: ArgumentType.STRING,
                meni: "EnableModeTypes",
                privzeta vrednost: "normalno",
              },
            },
          },
          // {
          // opcijska koda: 'setPhysics',
          // blockType: BlockType.COMMAND,
          // besedilo: formatMessage({
          // id: 'griffpatch.setPhysics',
          // privzeto: 'omogoči fiziko za sprite [oblika]',
          // opis: 'Omogoči fiziko za ta duh'
          // }),
          // argumenti: {
          // oblika: {
          // tip: ArgumentType.STRING,
          // meni: 'ShapeTypes',
          // defaultValue: 'kostum'
          // }
          // }
          // },
          // {
          // opcijska koda: 'setPhysicsAll',
          // blockType: BlockType.COMMAND,
          // besedilo: formatMessage({
          // id: 'griffpatch.setPhysicsAll',
          // privzeto: 'omogoči fiziko za vse sprite',
          // opis: 'Omogoči fiziko za vse sprite'
          // })
          // },
          //
          "---",

          {
            opcode: "doTick",
            blockType: BlockType.COMMAND,
            besedilo: formatMessage({
              id: "griffpatch.doTick",
              privzeto: "simulacija korakov",
              opis: "Zaženi eno kljukico fizikalne simulacije",
            }),
          },

          "---",

          {
            opcode: "setPosition",
            blockType: BlockType.COMMAND,
            besedilo: formatMessage({
              id: "griffpatch.setPosition",
              privzeto: "pojdi na x: [x] y: [y] [presledek]",
              opis: "Pozicija Sprite",
            }),
            argumenti: {
              x: {
                tip: ArgumentType.NUMBER,
                privzeta vrednost: 0,
              },
              y: {
                tip: ArgumentType.NUMBER,
                privzeta vrednost: 0,
              },
              presledek: {
                tip: ArgumentType.STRING,
                meni: "Vrste prostora",
                privzeta vrednost: "svet",
              },
            },
          },

          "---",

          // applyForce (target, ftype, x, y, dir, pow) {
          // applyAngForce (target, pow) {

          {
            opcode: "setVelocity",
            blockType: BlockType.COMMAND,
            besedilo: formatMessage({
              id: "griffpatch.setVelocity",
              privzeto: "nastavi hitrost na sx: [sx] sy: [sy]",
              opis: "Nastavi hitrost",
            }),
            argumenti: {
              sx: {
                tip: ArgumentType.NUMBER,
                privzeta vrednost: 0,
              },
              sy: {
                tip: ArgumentType.NUMBER,
                privzeta vrednost: 0,
              },
            },
          },
          {
            opcode: "changeVelocity",
            blockType: BlockType.COMMAND,
            besedilo: formatMessage({
              id: "griffpatch.changeVelocity",
              privzeto: "spremeni hitrost s sx: [sx] sy: [sy]",
              opis: "Spremeni hitrost",
            }),
            argumenti: {
              sx: {
                tip: ArgumentType.NUMBER,
                privzeta vrednost: 0,
              },
              sy: {
                tip: ArgumentType.NUMBER,
                privzeta vrednost: 0,
              },
            },
          },
          {
            opcode: "getVelocityX",
            besedilo: formatMessage({
              id: "griffpatch.getVelocityX",
              privzeto: "x hitrost",
              opis: "pridobite hitrost x",
            }),
            blockType: BlockType.REPORTER,
          },
          {
            opcode: "getVelocityY",
            besedilo: formatMessage({
              id: "griffpatch.getVelocityY",
              privzeto: "y hitrost",
              opis: "pridobite hitrost y",
            }),
            blockType: BlockType.REPORTER,
          },

          "---",

          {
            opcode: "applyForce",
            blockType: BlockType.COMMAND,
            besedilo: formatMessage({
              id: "griffpatch.applyForce",
              privzeto: "potisni s silo [sila] v smeri [dir]",
              opis: "Potisnite ta predmet v določeno smer",
            }),
            argumenti: {
              sila: {
                tip: ArgumentType.NUMBER,
                privzeta vrednost: 25,
              },
              dir: {
                tip: ArgumentType.NUMBER,
                privzeta vrednost: 0,
              },
            },
          },
          {
            opcode: "applyAngForce",
            blockType: BlockType.COMMAND,
            besedilo: formatMessage({
              id: "griffpatch.applyAngForce",
              default: "spin with force [force]",
              description: "Push this object in a given direction",
            }),
            arguments: {
              force: {
                type: ArgumentType.NUMBER,
                defaultValue: 500,
              },
            },
          },

          "---",

          {
            opcode: "setStatic",
            blockType: BlockType.COMMAND,
            text: formatMessage({
              id: "griffpatch.setStatic",
              default: "set fixed [static]",
              description: "Sets whether this block is static or dynamic",
            }),
            arguments: {
              static: {
                type: ArgumentType.STRING,
                menu: "StaticTypes",
                defaultValue: "static",
              },
            },
          },
          // {
          //     opcode: 'setDensity',
          //     blockType: BlockType.COMMAND,
          //     text: formatMessage({
          //         id: 'griffpatch.setDensity',
          //         default: 'set density [density]',
          //         description: 'Set the density of the object'
          //     }),
          //     arguments: {
          //         density: {
          //             type: ArgumentType.NUMBER,
          //             defaultValue: 1
          //         }
          //     }
          // },
          {
            opcode: "setProperties",
            blockType: BlockType.COMMAND,
            text: formatMessage({
              id: "griffpatch.setProperties",
              default:
                "set density [density] roughness [friction] bounce [restitution]",
              description: "Set the density of the object",
            }),
            arguments: {
              density: {
                type: ArgumentType.NUMBER,
                menu: "DensityTypes",
                defaultValue: 100,
              },
              friction: {
                type: ArgumentType.NUMBER,
                menu: "FrictionTypes",
                defaultValue: 50,
              },
              restitution: {
                type: ArgumentType.NUMBER,
                menu: "RestitutionTypes",
                defaultValue: 20,
              },
            },
          },
          // {
          //     opcode: 'pinSprite',
          //     blockType: BlockType.COMMAND,
          //     text: formatMessage({
          //         id: 'griffpatch.pinSprite',
          //         default: 'pin to world at sprite\'s x: [x] y: [y]',
          //         description: 'Pin the sprite'
          //     }),
          //     arguments: {
          //         x: {
          //             type: ArgumentType.NUMBER,
          //             defaultValue: 0
          //         },
          //         y: {
          //             type: ArgumentType.NUMBER,
          //             defaultValue: 0
          //         }
          //     }
          // },

          "---",

          {
            opcode: "getTouching",
            text: formatMessage({
              id: "griffpatch.getTouching",
              default: "touching [where]",
              description: "get the name of any sprites we are touching",
            }),
            blockType: BlockType.REPORTER,
            arguments: {
              where: {
                type: ArgumentType.STRING,
                menu: "WhereTypes",
                defaultValue: "any",
              },
            },
          },

          // Scene Scrolling -------------------

          "---",

          {
            opcode: "setScroll",
            blockType: BlockType.COMMAND,
            text: formatMessage({
              id: "griffpatch.setScroll",
              default: "set scroll x: [ox] y: [oy]",
              description: "Sets whether this block is static or dynamic",
            }),
            arguments: {
              ox: {
                type: ArgumentType.NUMBER,
                defaultValue: 0,
              },
              oy: {
                type: ArgumentType.NUMBER,
                defaultValue: 0,
              },
            },
          },
          {
            opcode: "changeScroll",
            blockType: BlockType.COMMAND,
            text: formatMessage({
              id: "griffpatch.changeScroll",
              default: "change scroll by x: [ox] y: [oy]",
              description: "Sets whether this block is static or dynamic",
            }),
            arguments: {
              ox: {
                type: ArgumentType.NUMBER,
                defaultValue: 0,
              },
              oy: {
                type: ArgumentType.NUMBER,
                defaultValue: 0,
              },
            },
          },
          {
            opcode: "getScrollX",
            text: formatMessage({
              id: "griffpatch.getScrollX",
              default: "x scroll",
              description: "get the x scroll",
            }),
            blockType: BlockType.REPORTER,
          },
          {
            opcode: "getScrollY",
            text: formatMessage({
              id: "griffpatch.getScrollY",
              default: "y scroll",
              description: "get the y scroll",
            }),
            blockType: BlockType.REPORTER,
          },

          // {
          //     opcode: 'getStatic',
          //     text: formatMessage({
          //         id: 'griffpatch.getStatic',
          //         default: 'Static?',
          //         description: 'get whether this sprite is static'
          //     }),
          //     blockType: BlockType.BOOLEAN
          // }
        ],

        menus: {
          StageTypes: this.STAGE_TYPE_MENU,
          SpaceTypes: this.SPACE_TYPE_MENU,
          WhereTypes: this.WHERE_TYPE_MENU,
          ShapeTypes: this.SHAPE_TYPE_MENU,
          EnableModeTypes: this.ENABLE_TYPES_TYPE_MENU,
          StaticTypes: this.STATIC_TYPE_MENU,
          FrictionTypes: this.FRICTION_TYPE_MENU,
          RestitutionTypes: this.RESTITUTION_TYPE_MENU,
          DensityTypes: this.DENSITY_TYPE_MENU,
        },
      };
    }

    get STAGE_TYPE_MENU() {
      return [
        { text: "boxed stage", value: STAGE_TYPE_OPTIONS.BOXED },
        { text: "open (with floor)", value: STAGE_TYPE_OPTIONS.FLOOR },
        { text: "open (no floor)", value: STAGE_TYPE_OPTIONS.OPEN },
      ];
    }

    get SPACE_TYPE_MENU() {
      return [
        { text: "in world", value: SPACE_TYPE_OPTIONS.WORLD },
        { text: "on stage", value: SPACE_TYPE_OPTIONS.STAGE },
        { text: "relative", value: SPACE_TYPE_OPTIONS.RELATIVE },
      ];
    }

    get WHERE_TYPE_MENU() {
      return [
        { text: "any", value: WHERE_TYPE_OPTIONS.ANY },
        { text: "feet", value: WHERE_TYPE_OPTIONS.FEET },
      ];
    }

    get SHAPE_TYPE_MENU() {
      return [
        { text: "this costume", value: SHAPE_TYPE_OPTIONS.COSTUME },
        { text: "this circle", value: SHAPE_TYPE_OPTIONS.CIRCLE },
        { text: "this polygon", value: SHAPE_TYPE_OPTIONS.SVG_POLYGON },
        { text: "all sprites", value: SHAPE_TYPE_OPTIONS.ALL },
      ];
    }

    get ENABLE_TYPES_TYPE_MENU() {
      return [
        { text: "normal", value: "normal" },
        { text: "precision", value: "bullet" },
      ];
    }

    get STATIC_TYPE_MENU() {
      return [
        { text: "free", value: "dynamic" },
        { text: "fixed in place", value: "static" },
        { text: "fixed (but can rotate)", value: "pinned" },
      ];
    }

    get DENSITY_TYPE_MENU() {
      return [
        { text: "very light", value: "25" },
        { text: "light", value: "50" },
        { text: "normal", value: "100" },
        { text: "heavy", value: "200" },
        { text: "very heavy", value: "400" },
      ];
    }

    get FRICTION_TYPE_MENU() {
      return [
        { text: "none", value: "0" },
        { text: "smooth", value: "20" },
        { text: "normal", value: "50" },
        { text: "rough", value: "75" },
        { text: "extremely rough", value: "100" },
      ];
    }

    get RESTITUTION_TYPE_MENU() {
      return [
        { text: "none", value: "0" },
        { text: "little", value: "10" },
        { text: "normal", value: "20" },
        { text: "quite bouncy", value: "40" },
        { text: "very bouncy", value: "70" },
        { text: "unstable", value: "100" },
      ];
    }

    /**
     * Play a drum sound for some number of beats.
     * @property {number} x - x offset.
     * @property {number} y - y offset.
     */
    doTick() {
      // args, util) {
      // this._playDrumForBeats(args.DRUM, args.BEATS, util);
      // if (util.runtime.audioEngine === null) return;
      // if (util.target.sprite.soundBank === null) return;

      // const dx = Cast.toNumber(args.x);
      // const dy = Cast.toNumber(args.y);

      // const allTargets = this.runtime.targets;
      // if (allTargets === null) return;
      // for (let i = 0; i < allTargets.length; i++) {
      //     const target = allTargets[i];
      //     if (!target.isStage) {
      //         target.setXY(target.x + dx, target.y + dy);
      //     }
      // }

      // util.target.setXY(util.target.x + dx, util.target.y + dy);

      // Matter.Engine.update(this.engine, 1000 / 30);
      this._checkMoved();

      // world.Step(1 / 30, 10, 10);
      world.Step(1 / 30, 10, 10);
      world.ClearForces();

      for (const targetID in bodies) {
        const body = bodies[targetID];
        const target = this.runtime.getTargetById(targetID);
        if (!target) {
          // Drop target from simulation
          world.DestroyBody(body);
          delete bodies[targetID];
          delete prevPos[targetID];
          continue;
        }

        const position = body.GetPosition();

        _setXY(
          target,
          position.x * zoom - _scroll.x,
          position.y * zoom - _scroll.y
        );
        if (target.rotationStyle === ROTATION_STYLE_ALL_AROUND) {
          target.setDirection(90 - body.GetAngle() / toRad);
        }

        prevPos[targetID] = { x: target.x, y: target.y, dir: target.direction };
      }
    }

    _checkMoved() {
      for (const targetID in bodies) {
        const body = bodies[targetID];
        const target = this.runtime.getTargetById(targetID);
        if (!target) {
          // Drop target from simulation
          world.DestroyBody(body);
          delete bodies[targetID];
          delete prevPos[targetID];
          continue;
        }

        const prev = prevPos[targetID];
        const fixedRotation = target.rotationStyle !== ROTATION_STYLE_ALL_AROUND;

        if (prev && (prev.x !== target.x || prev.y !== target.y)) {
          const pos = new b2Vec2(
            (target.x + _scroll.x) / zoom,
            (target.y + _scroll.y) / zoom
          );
          this._setPosition(body, pos);
          if (!fixedRotation) {
            body.SetAngle((90 - target.direction) * toRad);
          }
          body.SetAwake(true);
        } else if (!fixedRotation && prev && prev.dir !== target.direction) {
          body.SetAngle((90 - target.direction) * toRad);
          body.SetAwake(true);
        }
      }
    }

    /**
     * Play a drum sound for some number of beats.
     * @property {number} x - x offset.
     * @property {number} y - y offset.
     */
    setPhysicsAll() {
      const allTargets = this.runtime.targets;
      if (allTargets === null) return;
      for (let i = 0; i < allTargets.length; i++) {
        const target = allTargets[i];
        if (!target.isStage && !bodies[target.id]) {
          this.setPhysicsFor(target);
        }
      }
    }

    /**
     * Play a drum sound for some number of beats.
     * @param {object} args - the block arguments.
     * @param {object} util - utility object provided by the runtime.
     * @property {string} shape - the shape
     */
    setPhysics(args, util) {
      // this._playDrumForBeats(args.DRUM, args.BEATS, util);
      // if (util.runtime.audioEngine === null) return;
      // if (util.target.sprite.soundBank === null) return;

      // const dx = Cast.toNumber(args.x);
      // const dy = Cast.toNumber(args.y);

      if (args.shape === SHAPE_TYPE_OPTIONS.ALL) {
        this.setPhysicsAll();
        return;
      }

      const target = util.target;
      const body = this.setPhysicsFor(target, args.shape);
      if (body) {
        body.SetBullet(args.mode === "bullet");
      }
    }

    setPhysicsFor(target, shape) {
      const r = this.runtime.renderer;
      const drawable = r._allDrawables[target.drawableID];

      // Tell the Drawable about its updated convex hullPoints, if necessary.
      if (drawable.needsConvexHullPoints()) {
        const points = r._getConvexHullPointsForDrawable(target.drawableID);
        drawable.setConvexHullPoints(points);
      }

      // if (drawable._transformDirty) {
      //     drawable._calculateTransform();
      // }
      // const points = drawable._getTransformedHullPoints();
      //
      // const hullPoints = [];
      // for (const i in points) {
      //     hullPoints.push({x: points[i][0] - target.x, y: points[i][1] - target.y});
      // }

      const points = drawable._convexHullPoints;
      const scaleX = drawable.scale[0] / 100;
      const scaleY = drawable.scale[1] / -100; // Flip Y for hulls
      const offset = drawable.skin.rotationCenter;
      let allHulls = null;

      if (shape === SHAPE_TYPE_OPTIONS.CIRCLE) {
        fixDef.shape = new b2CircleShape();
        const size = drawable.skin.size;
        fixDef.shape.SetRadius(
          (size[0] * Math.abs(scaleX) + size[1] * Math.abs(scaleY)) / 4.0 / zoom
        );
        // fixDef.shape.SetRadius((drawable.getBounds().width / 2) / zoom);
      } else if (shape === SHAPE_TYPE_OPTIONS.SVG_POLYGON) {
        const svg = drawable._skin._svgRenderer._svgTag;

        // recurse through childNodes of type 'g', looking for type 'path'

        const hullPoints = [];
        if (svg) {
          this._fetchPolygonPointsFromSVG(
            svg,
            hullPoints,
            offset[0],
            offset[1],
            scaleX,
            scaleY
          );
        }

        _definePolyFromHull(hullPoints[0]);
        allHulls = hullPoints;
      } else {
        const hullPoints = [];
        for (const i in points) {
          hullPoints.push({
            x: (points[i][0] - offset[0]) * scaleX,
            y: (points[i][1] - offset[1]) * scaleY,
          });
        }

        _definePolyFromHull(hullPoints);
      }

      const fixedRotation = target.rotationStyle !== ROTATION_STYLE_ALL_AROUND;
      const body = _placeBody(
        target.id,
        target.x,
        target.y,
        fixedRotation ? 90 : target.direction
      );
      if (target.rotationStyle !== ROTATION_STYLE_ALL_AROUND) {
        body.SetFixedRotation(true);
      }

      if (allHulls) {
        for (let i = 1; i < allHulls.length; i++) {
          _definePolyFromHull(allHulls[i]);
          body.CreateFixture(fixDef);
        }
      }

      return body;
    }

    /**
     *
     * @param svg the svg element
     * @param {Array} hullPointsList array of points
     * @private
     */
    _fetchPolygonPointsFromSVG(svg, hullPointsList, ox, oy, scaleX, scaleY) {
      if (svg.tagName === "g" || svg.tagName === "svg") {
        if (svg.hasChildNodes()) {
          for (const node of svg.childNodes) {
            this._fetchPolygonPointsFromSVG(
              node,
              hullPointsList,
              ox,
              oy,
              scaleX,
              scaleY
            );
          }
        }
        return;
      }

      if (svg.tagName !== "path") {
        return;
      }
      // This is it boys! Get that svg data :)
      // <path xmlns="http://www.w3.org/2000/svg" d="M 1 109.7118 L 1 1.8097 L 60.3049 38.0516 L 117.9625 1.8097 L 117.9625 109.7118 L 59.8931 73.8817 Z "
      //  data-paper-data="{&quot;origPos&quot;:null}" stroke-width="2" fill="#9966ff"/>

      let fx;
      let fy;

      const hullPoints = [];
      hullPointsList.push(hullPoints);

      const tokens = svg.getAttribute("d").split(" ");
      for (let i = 0; i < tokens.length; ) {
        const token = tokens[i++];
        if (token === "M" || token === "L") {
          const x = Cast.toNumber(tokens[i++]);
          const y = Cast.toNumber(tokens[i++]);
          hullPoints.push({ x: (x - ox) * scaleX, y: (y - oy) * scaleY });
          if (token === "M") {
            fx = x;
            fy = y;
          }
        }
        if (token === "Z") {
          hullPoints.push({ x: (fx - ox) * scaleX, y: (fy - oy) * scaleY });
        }
      }
    }

    applyForce(args, util) {
      _applyForce(
        util.target.id,
        "Impulse",
        0,
        0,
        Cast.toNumber(args.dir),
        Cast.toNumber(args.force)
      );
    }

    applyAngForce(args, util) {
      let body = bodies[util.target.id];
      if (!body) {
        body = this.setPhysicsFor(util.target);
      }

      body.ApplyTorque(-Cast.toNumber(args.force));
    }

    setDensity(args, util) {
      let body = bodies[util.target.id];
      if (!body) {
        body = this.setPhysicsFor(util.target);
      }

      body.GetFixtureList().SetDensity(Cast.toNumber(args.density));
      body.ResetMassData();
    }

    setProperties(args, util) {
      let body = bodies[util.target.id];
      if (!body) {
        body = this.setPhysicsFor(util.target);
      }

      body.GetFixtureList().SetDensity(Cast.toNumber(args.density) / 100.0);
      body.GetFixtureList().SetFriction(Cast.toNumber(args.friction) / 100.0);
      body
        .GetFixtureList()
        .SetRestitution(Cast.toNumber(args.restitution) / 100.0);
      body.ResetMassData();
    }

    pinSprite(args, util) {
      if (!bodies[util.target.id]) {
        this.setPhysicsFor(util.target);
      }

      const x = Cast.toNumber(args.x);
      const y = Cast.toNumber(args.y);

      _createJointOfType(
        null,
        "Rotating",
        util.target.id,
        x,
        y,
        null,
        null,
        null
      );
    }

    /**
     * Set's the sprites position.
     * @param {object} args - the block arguments.
     * @param {object} util - utility object provided by the runtime.
     * @property {number} x - x offset.
     * @property {number} y - y offset.
     * @property {string} space - Space type (SPACE_TYPE_OPTIONS)
     */
    setPosition(args, util) {
      const x = Cast.toNumber(args.x);
      const y = Cast.toNumber(args.y);
      const body = bodies[util.target.id];

      switch (args.space) {
        case SPACE_TYPE_OPTIONS.STAGE:
          _setXY(util.target, x, y); // Position on stage (after scroll)
          if (body) {
            this._setPosition(
              body,
              new b2Vec2((x + _scroll.x) / zoom, (y + _scroll.y) / zoom)
            );
          }
          break;
        case SPACE_TYPE_OPTIONS.RELATIVE: {
          _setXY(util.target, util.target.x + x, util.target.x + y);
          if (body) {
            const pos = body.GetPosition();
            const pos2 = new b2Vec2(pos.x + x / zoom, pos.y + y / zoom);
            this._setPosition(body, pos2);
          }
          break;
        }
        default:
          _setXY(util.target, x - _scroll.x, y - _scroll.y);
          if (body) {
            this._setPosition(body, new b2Vec2(x / zoom, y / zoom));
          }
      }
    }

    _setPosition(body, pos2) {
      const md = pinned[body.uid];
      if (md) {
        world.DestroyJoint(md);
        pinned[body.uid] = _createJointOfType(
          null,
          "Rotating",
          body.uid,
          0,
          0,
          null,
          pos2.x * zoom,
          pos2.y * zoom
        );
      }
      body.SetPosition(pos2);
      // if (md) {
      //     pinned[body.uid] = _createJointOfType(null, 'Rotating', body.uid, 0, 0, null, null, null);
      // }
    }

    /**
     * Set the sprites velocity.
     * @param {object} args - the block arguments.
     * @param {object} util - utility object provided by the runtime.
     * @property {number} sx - speed x.
     * @property {number} sy - speed y.
     */
    setVelocity(args, util) {
      let body = bodies[util.target.id];
      if (!body) {
        body = this.setPhysicsFor(util.target);
      }

      body.SetAwake(true);

      const x = Cast.toNumber(args.sx);
      const y = Cast.toNumber(args.sy);
      const force = new b2Vec2(x, y);
      force.Multiply(30 / zoom);
      body.SetLinearVelocity(force);
    }

    /**
     * Change the sprites velocity.
     * @param {object} args - the block arguments.
     * @param {object} util - utility object provided by the runtime.
     * @property {number} sx - speed x.
     * @property {number} sy - speed y.
     */
    changeVelocity(args, util) {
      let body = bodies[util.target.id];
      if (!body) {
        body = this.setPhysicsFor(util.target);
      }

      body.SetAwake(true);

      const x = Cast.toNumber(args.sx);
      const y = Cast.toNumber(args.sy);
      const force = new b2Vec2(x, y);
      force.Multiply(30 / zoom);
      force.Add(body.GetLinearVelocity());
      body.SetLinearVelocity(force);
    }

    /**
     * Get the current tempo.
     * @param {object} args - the block arguments.
     * @param {object} util - utility object provided by the runtime.
     * @return {boolean} - the current tempo, in beats per minute.
     */
    getStatic(args, util) {
      const body = bodies[util.target.id];
      if (!body) {
        return false;
      }
      const type = body.GetType();
      return type === b2Body.b2_staticBody;
    }

    /**
     * Get the current tempo.
     * @param {object} args - the block arguments.
     * @param {object} util - utility object provided by the runtime.
     * @return {number} - the current x velocity.
     */
    getVelocityX(args, util) {
      const body = bodies[util.target.id];
      if (!body) {
        return 0;
      }
      const x = body.GetLinearVelocity().x;
      return (x * zoom) / 30;
    }

    /**
     * Get the current tempo.
     * @param {object} args - the block arguments.
     * @param {object} util - utility object provided by the runtime.
     * @return {boolean} - the current y velocity.
     */
    getVelocityY(args, util) {
      const body = bodies[util.target.id];
      if (!body) {
        return 0;
      }
      const y = body.GetLinearVelocity().y;
      return (y * zoom) / 30;
    }

    /**
     * Sets the static property
     * @param {object} args - the block arguments.
     * @param {object} util - utility object provided by the runtime.
     * @property {string} static - static or not
     */
    setStatic(args, util) {
      const target = util.target;
      let body = bodies[util.target.id];
      if (!body) {
        body = this.setPhysicsFor(target);
      }
      body.SetType(
        args.static === "static" ? b2Body.b2_staticBody : b2Body.b2_dynamicBody
      );

      const pos = new b2Vec2(
        (target.x + _scroll.x) / zoom,
        (target.y + _scroll.y) / zoom
      );
      const fixedRotation = target.rotationStyle !== ROTATION_STYLE_ALL_AROUND;
      body.SetPositionAndAngle(
        pos,
        fixedRotation ? 0 : (90 - target.direction) * toRad
      );

      if (args.static === "pinned") {
        // Find what's behind the sprite (pin to that)
        const point = new b2AABB();
        point.lowerBound.SetV(pos);
        point.upperBound.SetV(pos);
        let body2ID = null;
        world.QueryAABB((fixture) => {
          const body2 = fixture.GetBody();
          if (body2 !== body && fixture.TestPoint(pos.x, pos.y)) {
            body2ID = body2.uid;
            return false;
          }
          return true;
        }, point);

        pinned[target.id] = _createJointOfType(
          null,
          "Rotating",
          target.id,
          0,
          0,
          body2ID,
          null,
          null
        );
      } else {
        const pin = pinned[target.id];
        if (pin) {
          world.DestroyJoint(pin);
          // delete joints[pin.I];
          delete pinned[target.id];
        }
      }
    }

    /**
     * Sets the sprite offset
     * @param {object} args - the block arguments.
     * @property {number} ox - x offset.
     * @property {number} oy - y offset.
     */
    setScroll(args) {
      this._checkMoved();
      _scroll.x = Cast.toNumber(args.ox);
      _scroll.y = Cast.toNumber(args.oy);
      this._repositionBodies();
    }

    /**
     * Sets the sprite offset
     * @param {object} args - the block arguments.
     * @property {number} ox - x offset.
     * @property {number} oy - y offset.
     */
    changeScroll(args) {
      this._checkMoved();
      _scroll.x += Cast.toNumber(args.ox);
      _scroll.y += Cast.toNumber(args.oy);
      this._repositionBodies();
    }

    /**
     * Get the scroll x.
     * @return {number} - the current x velocity.
     */
    getScrollX() {
      return _scroll.x;
    }

    /**
     * Get the scroll x.
     * @return {number} - the current x velocity.
     */
    getScrollY() {
      return _scroll.y;
    }

    _repositionBodies() {
      for (const targetID in bodies) {
        const body = bodies[targetID];
        const target = this.runtime.getTargetById(targetID);
        if (target) {
          const position = body.GetPosition();
          _setXY(
            target,
            position.x * zoom - _scroll.x,
            position.y * zoom - _scroll.y
          );
          prevPos[targetID] = { x: target.x, y: target.y, dir: target.direction };
        }
      }
    }

    getTouching(args, util) {
      const target = util.target;
      const body = bodies[target.id];
      if (!body) {
        return "";
      }
      const where = args.where;
      let touching = "";
      const contacts = body.GetContactList();
      for (let ce = contacts; ce; ce = ce.next) {
        // noinspection JSBitwiseOperatorUsage
        if (ce.contact.m_flags & b2Contact.e_islandFlag) {
          continue;
        }
        if (
          ce.contact.IsSensor() === true ||
          ce.contact.IsEnabled() === false ||
          ce.contact.IsTouching() === false
        ) {
          continue;
        }
        const contact = ce.contact;
        const fixtureA = contact.GetFixtureA();
        const fixtureB = contact.GetFixtureB();
        const bodyA = fixtureA.GetBody();
        const bodyB = fixtureB.GetBody();

        // const myFix = touchingB ? fixtureA : fixtureB;

        const touchingB = bodyA === body;
        if (where !== "any") {
          const man = new Box2D.Collision.b2WorldManifold();
          contact.GetWorldManifold(man);
          // man.m_points
          // const mx = man.m_normal.x;
          // const my = man.m_normal.y;

          if (where === "feet") {
            // if (my > -0.6) {
            //     continue;
            // }

            const fixture = body.GetFixtureList();
            const y = man.m_points[0].y;
            if (
              y >
              fixture.m_aabb.lowerBound.y * 0.75 +
                fixture.m_aabb.upperBound.y * 0.25
            ) {
              continue;
            }

            // const lp = body.GetLocalPoint(man.m_points[0]).Normalize();
            // if (lp.y)
          }
        }

        const other = touchingB ? bodyB : bodyA;
        const uid = other.uid;
        const target2 = uid
          ? this.runtime.getTargetById(uid)
          : this.runtime.getTargetForStage();
        if (target2) {
          const name = target2.sprite.name;
          if (touching.length === 0) {
            touching = name;
          } else {
            touching += `,${name}`;
          }
        }
      }
      return touching;
    }

    /**
     * Sets the stage
     * @param {object} args - the block arguments.
     * @property {number} stageType - Stage Type.
     */
    setStage(args) {
      _setStageType(args.stageType);
    }

    /**
     * Sets the gravity
     * @param {object} args - the block arguments.
     * @property {number} gx - Gravity x.
     * @property {number} gy - Gravity y.
     */
    setGravity(args) {
      world.SetGravity(
        new b2Vec2(Cast.toNumber(args.gx), Cast.toNumber(args.gy))
      );
      for (const bodyID in bodies) {
        bodies[bodyID].SetAwake(true);
      }
    }
  }

  Scratch.extensions.register(new Scratch3Griffpatch());
})(Scratch);