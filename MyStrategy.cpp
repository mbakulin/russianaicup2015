#include "MyStrategy.h"
#include <algorithm>

#define PI 3.14159265358979323846
#define EPS 0.01
#define DIRECTION_EPS 0.05
#define OFFSET 0.1
#define _USE_MATH_DEFINES
#define SAFE 140
#define OPTIMIZATION_STEP 0.05
#define SMOOTHING_ITERATIONS 500
#define BONUS_PICKUP_START_ITERATION 250
#define N_OF_LOOKAHEAD_TILES 11
#define LOOKAHEAD_WAYPOINT_INDEX 2
#define MAX_DIST_TO_WAYPOINT_RECALC 50
#define CENTRIFUGAL_ACCELERACTION_BRAKE 0.37
#define CENTRIFUGAL_ACCELERACTION_ENGINE_POWER_DROP 0.3
#define CENTRIFUGAL_ACCELERATION_OILED 0.3
#define LOW_HP_LEVEL 20
#define LOW_HP_ACCELERATION_BRAKE 0.35
#define LOW_HP_ACCELERATION_ENGINE_POWER_DROP 0.2
#define LOW_HP_OILED_ACCELERATION 0.25
#define BRAKE_CURVATURE_LOOKAHEAD 3
#define NITRO_CURVATURE_LOOKAHEAD 9
#define MINIMUM_TICK_FROM_START_FOR_NITRO 180
#define MAX_SHOOTING_DISTANCE 2000
#define MAX_SHOOTING_DISTANCE_THRESHOLD 50
#define OIL_LOOKAHEAD_TILES_NUMBER 7
#define RECOVERY_TICKS 130
#define MIN_PROGRESS_SINCE_PREV_TICKS 10
#define MIN_TICKS_BEFORE_NEXT_RECOVERY 100
#define POS_LOOKBACK_FOR_RECOVERY 25
#define POSITION_HISTORY_SIZE 50
#define SHOOTING_MAX_TRAVEL_TIME_DELTA 5
#define ADDITIONAL_DIRECTION_WAYPOINT_DISTANCE 300
#define SPEED_THRESHOLD 3
#define SPEED_MULTIPLIER 50
#define MIN_DISTANCE_BETWEEN_LINKS 70
#define LOOKBACK_STEP_FOR_RACE_LINE_OPTIMIZATION 15
#define TILES_HISTORY_SIZE 4
#define MAX_TICKS_BETWEEN_PATH_UPDATES 50
#define MAX_VELOCITY 150
#define PATHFINDING_BRUTEFORCE_DEPTH 11
#define START_NITRO_LOOKAHEAD 100

#define TILE_WEIGHT 10
#define WAYPOINT_BONUS 20
#define SHARP_TURN_PENALTY 50
//#define STRAIGHT_BONUS 3
//#define LEFT_RIGHT_BONUS 5
#define STRAIGHT_BONUS 10
#define LEFT_RIGHT_BONUS 12
#define TURNAROUND_PENALTY 150
#define SAME_DIRECTION_BONUS 10
#define HAS_PURE_SCORE 55
#define HAS_HP 25
#define HAS_NITRO 35
#define HAS_OIL 15
#define HAS_PROJECTILE 15
#define CHANGED_MIND_PENALTY 10
#define STRAIGHT_START_BONUS 10
#define STRAGHT_START_LENGTH 100

#define PICKUP_DIST_TO_BONUS 250
#define BONUS_PICKUP_WIDTH 250
#define BONUS_PICUP_INNER_RADIUS 200
#define BONUS_PICKUP_OUTER_RADIUS 500

#define REPAIR_MASS 40
#define POINTS_MASS 80
#define NITRO_MASS 50
#define PROJECTILE_MASS 30
#define OIL_CANISTER_MASS 40

#include <cmath>
#include <cstdlib>

using namespace model;
using namespace std;


ChainLine::ChainLine(MyStrategy &strategy, const vector<intPoint> &tiles) : strategy(strategy), currentPosIndex(0) {
	links.resize(tiles.size());
	fixed.resize(tiles.size());
	masses.resize(tiles.size());
	for (int i = 0; i < tiles.size(); i++) {
		doublePoint newLink;
		links[i].x = (tiles[i].x + (double)0.5)*strategy.getGame().getTrackTileSize();
		links[i].y = (tiles[i].y + (double)0.5)*strategy.getGame().getTrackTileSize();
		fixed[i] = false;
		masses[i] = 1;
	}
}


void ChainLine::addShortcutsAndUpdateMasses() {
	vector<doublePoint> newLinks;
	newLinks = links;
	for (int i = 1; i < links.size() - 1; i++) {
		doublePoint start, middle, end;
		start = links[i - 1];
		middle = links[i];
		end = links[i + 1];

		double relativeAngleTo = middle.angle(start, end);

		//isDiagonal[i] = false;
		if ((fabs(relativeAngleTo - PI / 2) < DIRECTION_EPS)
			||
			(fabs(relativeAngleTo + PI / 2) < DIRECTION_EPS)) {
			doublePoint newPoint;
			newPoint = doublePoint::middle(start, end);
			newPoint = doublePoint::middle(newPoint, middle);
			newLinks[i] = newPoint;
		}
		else
			masses[i] = masses[i] / 1;
	} 

	/*doublePoint middle;
	middle = doublePoint::middle(newLinks[0], newLinks[1]);
	newLinks.insert(newLinks.begin() + 1, middle);
	fixed.insert(fixed.begin() + 1, false); */

	newLinks[0].x = strategy.getSelf().getX();
	newLinks[0].y = strategy.getSelf().getY();
	fixed[0] = true;

	links = newLinks;
}

void ChainLine::split() {
	int newSize = links.size() * 2 - 1;
	vector <doublePoint> newLinks;
	vector <bool> newFixed;
	vector <double> newMasses;
	int newCurrentPosIndex = 0;
	newLinks.reserve(newSize);
	newFixed.reserve(newSize);
	newMasses.reserve(newSize);

	for (int i = 0; i < links.size(); i++) {
		newLinks.push_back(links[i]);
		newFixed.push_back(fixed[i]);
		newMasses.push_back(masses[i]);
		if (i == currentPosIndex)
			newCurrentPosIndex = newLinks.size() - 1;
		if (i < links.size() - 1) {
			doublePoint newLink = doublePoint::middle(links[i], links[i + 1]);
			double mass = (masses[i] + masses[i + 1]) / 2;
			newLinks.push_back(newLink);
			newFixed.push_back(false);
			newMasses.push_back(mass);
		}
	}

	links = newLinks;
	fixed = newFixed;
	masses = newMasses;
	currentPosIndex = newCurrentPosIndex;

	int i = 0;
	while (i < links.size()) {
		if (!fixed[i] && !strategy.isOnTrack(links[i]))
			removePoint(i);
		else
			i++;
	}
}

void ChainLine::sparse() {
	int i = 1;
	while (i < links.size()) {
		if (links[i - 1].length(links[i]) < MIN_DISTANCE_BETWEEN_LINKS && !fixed[i]) {
			removePoint(i);
		}
		else
			i++;
	}
}

class Spring {
public:
	double defaultLength;
	double stiffness;

	Spring(doublePoint start, doublePoint finish, double stiffness = 0.2	) {
		defaultLength = start.length(finish);
		this->stiffness = stiffness;
	}

	Spring() {
		defaultLength = 1;
		this->stiffness = 0.2;
	}

	doublePoint force(doublePoint start, doublePoint end) {
		doublePoint result;
		double newLength = start.length(end);
		double forceModule = (newLength - defaultLength) * stiffness;
		result.x = forceModule * (end.x - start.x) / newLength;
		result.y = forceModule * (end.y - start.y) / newLength;
		return result;
	}
};

void ChainLine::optimizeLine()
{
	vector <doublePoint> force;
	vector <doublePoint> velocity;
	vector <Spring> spring;
	force.resize(links.size());
	velocity.resize(links.size());
	spring.resize(links.size() - 1);

	for (int i = 0; i < velocity.size(); i++)
	{
		velocity[i].x = 0;
		velocity[i].y = 0;
		if (i != spring.size())
			spring[i] = Spring(links[i], links[i + 1]);
	}

	vector <pair <Bonus, int> >  bonusWithLinkIndex;
	vector <vector <vector <int > > > minLinkIndex;
	vector <vector <vector <double > > > minDistanceToLink;
	minLinkIndex.resize(strategy.getWorld().getWidth());
	minDistanceToLink.resize(strategy.getWorld().getWidth());
	for (int x = 0; x < minLinkIndex.size(); x++) {
		minLinkIndex[x].resize(strategy.getWorld().getHeight());
		minDistanceToLink[x].resize(strategy.getWorld().getHeight());
		for (int y = 0; y < minLinkIndex[x].size(); y++) {
			for (int k = 0; k < strategy.getBonuses(x, y).size(); k++) {
				minLinkIndex[x][y].push_back(-1);
				minDistanceToLink[x][y].push_back(10000);
			}
		}
	}

	for (int iteration = 0; iteration < SMOOTHING_ITERATIONS; iteration++) {
		for (int j = 0; j < force.size(); j++) {
			force[j].x = 0;
			force[j].y = 0;
		}
		if (!fixed[1])
		{
			/*doublePoint first;
			first = links[1];
			doublePoint springNormal;
			springNormal.x = -(first.y - links[0].y);
			springNormal.y = (first.x - links[0].x);
			double absoluteAngle = atan2(first.y - links[0].y, first.x - links[0].x);
			double angleDiff = currentAngle - absoluteAngle;
			while (angleDiff > PI) {
				angleDiff -= 2.0 * PI;
			}

			while (angleDiff < -PI) {
				angleDiff += 2.0 * PI;
			}
			
			double multiplier = angleDiff;
			force[1] = force[1] + multiplier*springNormal; */
		}
		for (int i = 1; i < links.size() - 1; i++) {
			doublePoint start, middle, end;
			start = links[i - 1];
			middle = links[i];
			end = links[i + 1];

			doublePoint springForce1 = spring[i - 1].force(start, middle) * 1;
			doublePoint springForce2 = spring[i].force(middle, end) * 1;

			force[i] = force[i] -springForce1 + springForce2;
			
			double relativeAngleTo = middle.angle(start, end);

			//if (!(fabs(fabs(relativeAngleTo) - PI) < PI / 30)) {
			double multiplier = relativeAngleTo > 0 ? PI - relativeAngleTo : -PI - relativeAngleTo;
			doublePoint springNormal1;
			doublePoint springNormal2;
			springNormal1.x = -(middle.y - start.y);
			springNormal1.y = (middle.x - start.x);
			springNormal2.x = -(end.y - middle.y);
			springNormal2.y = (end.x - middle.x);
			force[i - 1] = force[i - 1] - multiplier * springNormal1;
			force[i] = force[i] + multiplier * (springNormal2 + springNormal1);
			force[i + 1] = force[i + 1] - multiplier*springNormal2;
			//}

			/* if (iteration == BONUS_PICKUP_START_ITERATION && masses[i] == 1 && !fixed[i] && i > currentPosIndex) {
				int x, y;
				x = links[i].x / strategy.getGame().getTrackTileSize();
				y = links[i].y / strategy.getGame().getTrackTileSize();
				const vector <Bonus> &bonusesInTile = strategy.getBonuses(x, y);
				bool alreadyLinked = false;
				for (int j = 0; j < bonusesInTile.size(); j++) {
					for (int k = 0; k < bonusWithLinkIndex.size(); k++) {
						if (bonusWithLinkIndex[k].first.getId() == bonusesInTile[j].getId()) {
							alreadyLinked = true;
							break;
						}
					}
					if (!alreadyLinked)
					{
						double linkDistanceToBonus = links[i].length(doublePoint(bonusesInTile[j]));
						if (linkDistanceToBonus < minDistanceToLink[x][y][j]) {
							minDistanceToLink[x][y][j] = linkDistanceToBonus;
							minLinkIndex[x][y][j] = i;
						}
					}
				}
			} */
		}		

		if (iteration == BONUS_PICKUP_START_ITERATION) {
			for (int i = currentPosIndex + 1; i < links.size(); i++) {
				int x, y;
				x = links[i].x / strategy.getGame().getTrackTileSize();
				y = links[i].y / strategy.getGame().getTrackTileSize();
				const vector <Bonus> &bonusesInTile = strategy.getBonuses(x, y);
				for (int j = 0; j < bonusesInTile.size(); j++) {
					doublePoint bonusCoords(bonusesInTile[j]);
					if (links[i].length(bonusCoords) < minDistanceToLink[x][y][j]) {
						minDistanceToLink[x][y][j] = links[i].length(bonusCoords);
						minLinkIndex[x][y][j] = i;
					}
				}
			}
			for (int i = currentPosIndex + 1; i < links.size(); i++) {
				if (masses[i] == 1 && !fixed[i]) {
					int x, y;
					x = links[i].x / strategy.getGame().getTrackTileSize();
					y = links[i].y / strategy.getGame().getTrackTileSize();
					const vector <Bonus> &bonusesInTile = strategy.getBonuses(x, y);
					int bestBonusIndex = 0;
					double bestMass = 0;
					double bestDistance = 10000;
					for (int j = 0; j < bonusesInTile.size(); j++) {
						if (i == minLinkIndex[x][y][j]) {
							doublePoint bonusCoords(bonusesInTile[j]);
							if (links[i].length(bonusCoords) < PICKUP_DIST_TO_BONUS) {
								switch (bonusesInTile[j].getType()) {
								case REPAIR_KIT:
									if (strategy.getSelf().getDurability() < 0.5) {
										if (REPAIR_MASS > bestMass) {
											bestMass = REPAIR_MASS;
											bestBonusIndex = j;
											bestDistance = links[i].length(bonusCoords);
										}
										else if (REPAIR_MASS == bestMass) {
											if (bestDistance > links[i].length(bonusCoords)) {
												bestMass = REPAIR_MASS;
												bestBonusIndex = j;
												bestDistance = links[i].length(bonusCoords);
											}
										}
									}
									break;
								case NITRO_BOOST:
									if (NITRO_MASS > bestMass) {
										bestMass = NITRO_MASS;
										bestBonusIndex = j;
										bestDistance = links[i].length(bonusCoords);
									}
									else if (NITRO_MASS == bestMass) {
										if (bestDistance > links[i].length(bonusCoords)) {
											bestMass = NITRO_MASS;
											bestBonusIndex = j;
											bestDistance = links[i].length(bonusCoords);
										}
									}
									break;
								case AMMO_CRATE:
									if (PROJECTILE_MASS > bestMass) {
										bestMass = PROJECTILE_MASS;
										bestBonusIndex = j;
									}
									else if (PROJECTILE_MASS == bestMass) {
										if (bestDistance > links[i].length(bonusCoords)) {
											bestMass = PROJECTILE_MASS;
											bestBonusIndex = j;
											bestDistance = links[i].length(bonusCoords);
										}
									}
									break;
								case PURE_SCORE:
									if (POINTS_MASS > bestMass) {
										bestMass = POINTS_MASS;
										bestBonusIndex = j;
									}
									else if (POINTS_MASS == bestMass) {
										if (bestDistance > links[i].length(bonusCoords)) {
											bestMass = POINTS_MASS;
											bestBonusIndex = j;
											bestDistance = links[i].length(bonusCoords);
										}
									}
									break;
								case OIL_CANISTER:
									if (OIL_CANISTER_MASS > bestMass) {
										bestMass = OIL_CANISTER_MASS;
										bestBonusIndex = j;
									}
									else if (OIL_CANISTER_MASS == bestMass) {
										if (bestDistance > links[i].length(bonusCoords)) {
											bestMass = OIL_CANISTER_MASS;
											bestBonusIndex = j;
											bestDistance = links[i].length(bonusCoords);
										}
									}
									break;
								default:
									bestMass = 5;
									bestBonusIndex = j;
									bestDistance = bestDistance = links[i].length(bonusCoords);
								}
							}
						}
					}
					if (bestMass > 0) {
						if (masses[i - 1] < bestMass) {
							links[i] = bonusesInTile[bestBonusIndex];
							masses[i] = bestMass;
							masses[i - 1] = 1;
						}
					}
				}
			}
		}	

		double maxForce = 0;
		/* for (int i = 1; i < force.size(); i++) {
			maxForce = fmax(maxForce, force[i].length());// *(links.size() - i) / links.size());
		}*/
		// if (maxForce > EPS) {
		for (int i = 0; i < links.size(); i++) {
			if (!fixed[i]) {
				//doublePoint scaledForce = OPTIMIZATION_STEP * force[i] *(links.size() - i)/ (maxForce * links.size());
				doublePoint scaledForce = OPTIMIZATION_STEP * force[i]; // maxForce;
				velocity[i] = velocity[i] * 0.9 + scaledForce / masses[i];
			}
			else {
				velocity[i].x = 0;
				velocity[i].y = 0;
			}
			if (velocity[i].length() > MAX_VELOCITY) {
				velocity[i] = velocity[i] / velocity[i].length() * MAX_VELOCITY;
			}
		}
		//}
		for (int i = 0; i < links.size(); i++) {
			if (strategy.isOnTrack(links[i] + velocity[i])) {
				links[i] = links[i] + velocity[i];
			}
			else {
				velocity[i] = velocity[i] / 2;
			}
		}
	}
	
#ifdef PRINT_DEBUG
	strategy.debug.beginPost();
	for (int i = 0; i < links.size(); i++) {
		strategy.debug.fillCircle(links[i].x, links[i].y, 10, 0xFF0000);		
	}	
	strategy.debug.endPost();
#endif

	for (int i = 1; i < links.size() - 1; i++) {
		if (!fixed[i]) {
			doublePoint newPoint;
			newPoint = doublePoint::middle(links[i - 1], links[i + 1]);
			newPoint = doublePoint::middle(newPoint, links[i]);
			if (strategy.isOnTrack(newPoint)) {
				links[i] = newPoint;
			}
		}
	}
	for (int i = 1; i < links.size() - 1; i++) {
		if (!fixed[i]) {
			doublePoint newPoint;
			newPoint = doublePoint::middle(links[i - 1], links[i + 1]);
			newPoint = doublePoint::middle(newPoint, links[i]);
			if (strategy.isOnTrack(newPoint)) {
				links[i] = newPoint;
			}
		}
#ifdef PRINT_DEBUG
		if (masses[i] > 1)
			strategy.debug.fillCircle(links[i].x, links[i].y, 20, 0xFF0000);
		else
			strategy.debug.fillCircle(links[i].x, links[i].y, 10, 0x000000);
#endif
	}
#ifdef PRINT_DEBUG
	//strategy.debug.endPost();
#endif
}

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
	self_ptr = &self;
	world_ptr = &world;
	game_ptr = &game;
	move_ptr = &move;

	if (pathScore.size() == 0) {
		runPathfinding();
		initPrevTiles();
		bonuses.resize(world_ptr->getWidth());
		bonusHasAssignedLink.resize(bonuses.size());
		for (int i = 0; i < bonuses.size(); i++) {
			bonuses[i].resize(world_ptr->getHeight());
			bonusHasAssignedLink[i].resize(bonuses[i].size());
		}
	}

	if (world_ptr->getTick() < game_ptr->getInitialFreezeDurationTicks()) {
		move.setEnginePower(1);
		return;
	}

	updateBonuses();
	intPoint currentTile;
	currentTile.x = self_ptr->getX() / game_ptr->getTrackTileSize();
	currentTile.y = self_ptr->getY() / game_ptr->getTrackTileSize();

	if (!(currentTile == previousTiles.back())) {
		runPathfinding();
		if (std::find(previousTiles.begin(), previousTiles.end(), currentTile) != previousTiles.end()) {
			previousTiles.clear();
		}
		previousTiles.push_back(currentTile);
		while (previousTiles.size() > TILES_HISTORY_SIZE) {
			previousTiles.erase(previousTiles.begin());
		}
		updatePath(true);
	}

	if (canShoot()) {
		move.setThrowProjectile(true);
	}

	if (recoveryMode) {
		if (world_ptr->getTick() - recoveryModeStartTick < RECOVERY_TICKS) {
			move.setEnginePower(-1);
			nextWaypoint = getNextWaypoint();
			double angleToWaypoint = self.getAngleTo(nextWaypoint.x, nextWaypoint.y);
			move.setWheelTurn(-angleToWaypoint * 5);
			return;
		}
		else if (hypot(self_ptr->getSpeedX(), self_ptr->getSpeedY()) > 2) {
			move.setBrake(true);
			move.setEnginePower(1);
		}
		else {
			posOnPrevTicks.clear();
			recoveryModeEndTick = world_ptr->getTick();
			recoveryMode = false;
			previousTiles.clear();
			previousTiles.push_back(currentTile);
		}
		return;
	}
	else {
		if (world_ptr->getTick() >= 180) {
			if (self_ptr->getDurability() != 0) {
				doublePoint position(self_ptr->getX(), self_ptr->getY());
				posOnPrevTicks.insert(posOnPrevTicks.begin(), position);
				if (posOnPrevTicks.size() > POSITION_HISTORY_SIZE) {
					posOnPrevTicks.pop_back();
				}
				if (posOnPrevTicks.size() > POS_LOOKBACK_FOR_RECOVERY) {
					doublePoint prevPosition = posOnPrevTicks[POS_LOOKBACK_FOR_RECOVERY];
					if ((position - prevPosition).length() < MIN_PROGRESS_SINCE_PREV_TICKS
						&&
						world_ptr->getTick() - recoveryModeEndTick > MIN_TICKS_BEFORE_NEXT_RECOVERY) {
						recoveryMode = true;
						recoveryModeStartTick = world_ptr->getTick();
						previousTiles.clear();
						previousTiles.push_back(currentTile);
						updatePath(true);
					}
				}
			}
			else
			{
				posOnPrevTicks.clear();
			}
		}
		nextWaypoint = getNextWaypoint();
	}

	double angleToWaypoint = self.getAngleTo(nextWaypoint.x, nextWaypoint.y);
	double speedModule = hypot(self.getSpeedX(), self.getSpeedY());
	/* if (fabs(speedModule) > 2) {
		doublePoint speed;
		speed.x = self_ptr->getSpeedX();
		speed.y = self_ptr->getSpeedY();
		double speedAngle = self.getAngleTo(self.getX() + speed.x, self.getY() + speed.y);
		angleToWaypoint = angleToWaypoint - speedAngle;

		while (angleToWaypoint > PI) {
			angleToWaypoint -= 2.0 * PI;
		}

		while (angleToWaypoint < -PI) {
			angleToWaypoint += 2.0 * PI;
		}
	} */

	move.setWheelTurn(angleToWaypoint * 5);
	move.setEnginePower(1);

	int brakeCurvatureLookAhead = BRAKE_CURVATURE_LOOKAHEAD;
	if (speedModule > 30) {
		brakeCurvatureLookAhead += 1;
	}

	double centrifugalAcceleration = speedModule*speedModule / minCurvature[brakeCurvatureLookAhead];
	if (self_ptr->getDurability() * 100 < LOW_HP_LEVEL) {
		if (speedModule*speedModule / minCurvature[brakeCurvatureLookAhead] > LOW_HP_ACCELERATION_BRAKE) {
			move.setBrake(true);
		}
		else if (self_ptr->getRemainingOiledTicks() > 0 && centrifugalAcceleration > LOW_HP_OILED_ACCELERATION) {
			move.setBrake(true);
		}

		if (speedModule*speedModule / minCurvature[brakeCurvatureLookAhead] > LOW_HP_ACCELERATION_ENGINE_POWER_DROP) {
			//move.setEnginePower(0.3);
		}
	}
	else {
#ifdef PRINT_DEBUG
		char *buf = new char[Debug::BUF_SIZE];
		sprintf(buf, "\"%lf %lf\"", centrifugalAcceleration, minCurvature[BRAKE_CURVATURE_LOOKAHEAD]);
		debug.text(self.getX(),self.getY() + 100, buf, 0);
		delete buf;
		debug.endPost();
#endif
		if (centrifugalAcceleration > CENTRIFUGAL_ACCELERACTION_BRAKE) {
			move.setBrake(true);
		}
		else if (self_ptr->getRemainingOiledTicks() > 0 && centrifugalAcceleration > CENTRIFUGAL_ACCELERATION_OILED) {
			move.setBrake(true);
		}
		if (speedModule*speedModule / minCurvature[BRAKE_CURVATURE_LOOKAHEAD] > CENTRIFUGAL_ACCELERACTION_ENGINE_POWER_DROP) {
			//move.setEnginePower(0.3);
		}
	}

	if (minCurvature[NITRO_CURVATURE_LOOKAHEAD] > 4000) {		
		move.setUseNitro(true);
	}	

	if (move_ptr->isBrake()) {
		if (spillOil()) {
			move.setSpillOil(true);
		}
	}

	/* if (world_ptr->getTick() == game_ptr->getInitialFreezeDurationTicks()) {
		bool useNitroOnStart = true;
		Direction currenDirection = world_ptr->getStartingDirection();
		int deltaX = 0, deltaY = 0;
		switch (currenDirection) {
		case UP:
			deltaY = -1;
			break;
		case DOWN:
			deltaY = 1;
			break;
		case LEFT:
			deltaX = -1;
			break;
		case RIGHT:
			deltaX = 1;
			break;
		default:
			break;
		}
		for (int i = 1; i < START_NITRO_LOOKAHEAD; i++ ) {
			if (currentPath[i].x - currentPath[i - 1].x != deltaX
				||
				currentPath[i].y - currentPath[i - 1].y != deltaY) {
				useNitroOnStart = false;
				break;
			}
		}
		move.setUseNitro(useNitroOnStart);
	} */
}

MyStrategy::MyStrategy() {
	pathScore.resize(0);
	recoveryModeStartTick = 0;
	recoveryModeEndTick = 0;
	recoveryMode = false;
}

bool MyStrategy::canShoot() {
	if (self_ptr->getProjectileCount() == 0) {
		return false;
	}
	double angle = self_ptr->getAngle();
	double projectileSpeedModule;
	doublePoint projectileSpeed;
	if (self_ptr->getType() == JEEP) {
		projectileSpeedModule = game_ptr->getTireInitialSpeed();
	}
	else {
		projectileSpeedModule = game_ptr->getWasherInitialSpeed();
	}
	projectileSpeed.x = projectileSpeedModule * cos(angle);
	projectileSpeed.y = projectileSpeedModule * sin(angle);
	vector<Car> cars = world_ptr->getCars();
	for (int i = 0; i * projectileSpeedModule < MAX_SHOOTING_DISTANCE; i++) {
		doublePoint projectilePosition;
		projectilePosition.x = self_ptr->getX() + projectileSpeed.x * i;
		projectilePosition.y = self_ptr->getY() + projectileSpeed.y * i;
		if (self_ptr->getType() == JEEP && !isOnTrack(projectilePosition)) {
			return false;
		}
		for (int j = 0; j < cars.size(); j++) {
			if (cars[j].getId() == self_ptr->getId()) {
				continue;
			}
			doublePoint carPosition;
			carPosition.x = cars[j].getX() + cars[j].getSpeedX() * i;
			carPosition.y = cars[j].getY() + cars[j].getSpeedY() * i;
			if (carPosition.length(projectilePosition) < MAX_SHOOTING_DISTANCE_THRESHOLD) {
				if (cars[j].isTeammate())
					return false;
				else if (cars[j].getDurability() == 0) {
					return false;
				}
				else if (cars[j].isFinishedTrack()) {
					return false;
				}
				else
					return true;
			}
		}
	}
	return false;
}

bool MyStrategy::spillOil() {
	doublePoint oilPos;
	oilPos.x = self_ptr->getX() + cos(self_ptr->getAngle() - PI) * (self_ptr->getHeight() / 2 + game_ptr->getOilSlickRadius() / 2 + game_ptr->getOilSlickInitialRange());
	oilPos.y = self_ptr->getY() + sin(self_ptr->getAngle() - PI) * (self_ptr->getHeight() / 2 + game_ptr->getOilSlickRadius() / 2 + game_ptr->getOilSlickInitialRange());
	intPoint myTile;
	myTile.x = oilPos.x / game_ptr->getTrackTileSize();
	myTile.y = oilPos.y / game_ptr->getTrackTileSize();
	bool result = false;
	for (int i = 2; i < currentPath.size(); i++) {
		if (myTile == currentPath[i]) {
			return false;
		}
	}
	const vector <Car>& cars = world_ptr->getCars();
	for (int i = 0; i < cars.size(); i++) {
		if (cars[i].isTeammate())
			continue;		
		if (cars[i].getDurability() < EPS)
			continue;
		if (cars[i].isFinishedTrack())
			continue;
		int w = cars[i].getNextWaypointIndex();
		intPoint currentTile;
		currentTile.x = cars[i].getX() / game_ptr->getTrackTileSize();
		currentTile.y = cars[i].getY() / game_ptr->getTrackTileSize();

		vector <intPoint> tiles;

		tiles.push_back(currentTile);

		if (currentTile.x != myTile.x || currentTile.y != myTile.y)
		{
			while (tiles.size() != OIL_LOOKAHEAD_TILES_NUMBER)
			{
				currentTile = tiles[tiles.size() - 1];
				intPoint nextTile = currentTile;

				if (currentTile == myTile && tiles.size() > 2)
					return true;

				if (pathScore[w][currentTile.x][currentTile.y] == 0) {
					w = (w + 1) % nOfWaypoints;
					continue;
				}

				if (canGo(currentTile, RIGHT) && pathScore[w][currentTile.x + 1][currentTile.y] < pathScore[w][currentTile.x][currentTile.y]) {
					nextTile.x++;
				}
				else if (canGo(currentTile, LEFT) && pathScore[w][currentTile.x - 1][currentTile.y] < pathScore[w][currentTile.x][currentTile.y]) {
					nextTile.x--;
				}
				else if (canGo(currentTile, UP) && pathScore[w][currentTile.x][currentTile.y - 1] < pathScore[w][currentTile.x][currentTile.y]) {
					nextTile.y--;
				}
				else if (canGo(currentTile, DOWN) && pathScore[w][currentTile.x][currentTile.y + 1] < pathScore[w][currentTile.x][currentTile.y]) {
					nextTile.y++;
				}
				tiles.push_back(nextTile);
			}
		}
		else {
			doublePoint myPos, opponentPos, speedPos;
			doublePoint opponentSpeed;
			opponentSpeed.x = cars[i].getSpeedX();
			opponentSpeed.y = cars[i].getSpeedY();
			if (opponentSpeed.length() > 2) {
				opponentPos.x = cars[i].getX();
				opponentPos.y = cars[i].getY();
				speedPos = opponentPos + opponentSpeed;
				if (fabs(fabs(speedPos.angle(opponentPos, myPos) - PI)) < PI / 6) {
					return true;
				}
			}
		}
	}
	return result;
}

bool MyStrategy::canGo(intPoint point, Direction direction) {
	TileType tileType = world_ptr->getTilesXY()[point.x][point.y];
	switch (direction) {
		case _UNKNOWN_DIRECTION_:
			return true;
		case LEFT:
			if (point.x <= 0)
				return false;
			switch (tileType)
			{
			case HORIZONTAL:
			case RIGHT_TOP_CORNER:
			case RIGHT_BOTTOM_CORNER:
			case LEFT_HEADED_T:
			case TOP_HEADED_T:
			case BOTTOM_HEADED_T:
			case CROSSROADS:
				return true;
			case UNKNOWN: {
				intPoint nextPoint = point;
				nextPoint.x -= 1;
				TileType nextTileType = world_ptr->getTilesXY()[nextPoint.x][nextPoint.y];
				switch (nextTileType) {
				case UNKNOWN:
					return true;
				default:
					return canGo(nextPoint, RIGHT);
				}
			}
			default:
				return false;
			}
		case RIGHT:
			if (point.x >= width - 1)
				return false;
			switch (tileType)
			{
			case HORIZONTAL:
			case LEFT_TOP_CORNER:
			case LEFT_BOTTOM_CORNER:
			case RIGHT_HEADED_T:
			case TOP_HEADED_T:
			case BOTTOM_HEADED_T:
			case CROSSROADS:
				return true;
			case UNKNOWN: {
				intPoint nextPoint = point;
				nextPoint.x += 1;
				TileType nextTileType = world_ptr->getTilesXY()[nextPoint.x][nextPoint.y];
				switch (nextTileType) {
				case UNKNOWN:
					return true;
				default:
					return canGo(nextPoint, LEFT);
				}
			}
			default:
				return false;
			}
		case UP:
			if (point.y <= 0)
				return false;
			switch (tileType)
			{
			case VERTICAL:
			case LEFT_BOTTOM_CORNER:
			case RIGHT_BOTTOM_CORNER:
			case LEFT_HEADED_T:
			case TOP_HEADED_T:
			case RIGHT_HEADED_T:
			case CROSSROADS:
				return true;
			case UNKNOWN: {
				intPoint nextPoint = point;
				nextPoint.y -= 1;
				TileType nextTileType = world_ptr->getTilesXY()[nextPoint.x][nextPoint.y];
				switch (nextTileType) {
				case UNKNOWN:
					return true;
				default:
					return canGo(nextPoint, DOWN);
				}
			}
			default:
				return false;
			}
		case DOWN:
			if (point.y >= height - 1)
				return false;
			switch (tileType)
			{
			case VERTICAL:
			case LEFT_TOP_CORNER:
			case RIGHT_TOP_CORNER:
			case LEFT_HEADED_T:
			case BOTTOM_HEADED_T:
			case RIGHT_HEADED_T:
			case CROSSROADS:
				return true;
			case UNKNOWN: {
				intPoint nextPoint = point;
				nextPoint.y += 1;
				TileType nextTileType = world_ptr->getTilesXY()[nextPoint.x][nextPoint.y];
				switch (nextTileType) {
				case UNKNOWN:
					return true;
				default:
					return canGo(nextPoint, UP);
				}
			}
			default:
				return false;
			}
		default:
			return false;
	}
}

void MyStrategy::recursiveFindPath(intPoint start, int w, int score) {
	pathScore[w][start.x][start.y] = score;

	if (canGo(start, UP)) {
		intPoint nextPoint;
		nextPoint.x = start.x;
		nextPoint.y = start.y - 1;
		if (pathScore[w][nextPoint.x][nextPoint.y] > score + TILE_WEIGHT) {
			recursiveFindPath(nextPoint, w, score + TILE_WEIGHT);
		}
	}

	if (canGo(start, DOWN)) {
		intPoint nextPoint;
		nextPoint.x = start.x;
		nextPoint.y = start.y + 1;
		if (pathScore[w][nextPoint.x][nextPoint.y] > score + TILE_WEIGHT) {
			recursiveFindPath(nextPoint, w, score + TILE_WEIGHT);
		}
	}

	if (canGo(start, LEFT)) {
		intPoint nextPoint;
		nextPoint.x = start.x - 1;
		nextPoint.y = start.y;
		if (pathScore[w][nextPoint.x][nextPoint.y] > score + TILE_WEIGHT) {
			recursiveFindPath(nextPoint, w, score + TILE_WEIGHT);
		}
	}

	if (canGo(start, RIGHT)) {
		intPoint nextPoint;
		nextPoint.x = start.x + 1;
		nextPoint.y = start.y;
		if (pathScore[w][nextPoint.x][nextPoint.y] > score + TILE_WEIGHT) {
			recursiveFindPath(nextPoint, w, score + TILE_WEIGHT);
		}
	}
}

void MyStrategy::runPathfinding() {
	nOfWaypoints = world_ptr->getWaypoints().size();
	width = world_ptr->getWidth();
	height = world_ptr->getHeight();
	pathScore.resize(nOfWaypoints);
	for (int w = 0; w < nOfWaypoints; w++) {
		pathScore[w].resize(width);
		for (int x = 0; x < width; x++) {
			pathScore[w][x].resize(height);
			for (int y = 0; y < height; y++) {
				pathScore[w][x][y] = INT_MAX;
			}
		}
		intPoint target;
		target.x = world_ptr->getWaypoints()[w][0];
		target.y = world_ptr->getWaypoints()[w][1];

		recursiveFindPath(target, w, 0);
	}
}

doublePoint MyStrategy::getNextWaypoint() {
	doublePoint result;
	
	updatePath(false);

	ChainLine racingLine(*this, currentPath);
	racingLine.addShortcutsAndUpdateMasses();
	
	if (!recoveryMode) {
		doublePoint speedVector;
		speedVector.x = self_ptr->getSpeedX();
		speedVector.y = self_ptr->getSpeedY();
		double speedAngle = atan2(speedVector.y, speedVector.x);
		doublePoint newLink;
		if (posOnPrevTicks.size() < LOOKBACK_STEP_FOR_RACE_LINE_OPTIMIZATION || speedVector.length() < 2) {
			newLink.x = racingLine[0].x - ADDITIONAL_DIRECTION_WAYPOINT_DISTANCE * cos(self_ptr->getAngle());
			newLink.y = racingLine[0].y - ADDITIONAL_DIRECTION_WAYPOINT_DISTANCE * sin(self_ptr->getAngle());
			racingLine.insertPoint(newLink, 0, true);
		}
		else {
			for (int i = LOOKBACK_STEP_FOR_RACE_LINE_OPTIMIZATION; i < posOnPrevTicks.size(); i += LOOKBACK_STEP_FOR_RACE_LINE_OPTIMIZATION) {
				newLink = posOnPrevTicks[i];
				racingLine.insertPoint(newLink, 0, true);
			}			
		}
		/*if (speedVector.length() > SPEED_THRESHOLD) {
			newLink.x = racingLine[0].x - fmin(speedVector.length() * SPEED_MULTIPLIER, ADDITIONAL_DIRECTION_WAYPOINT_DISTANCE) * cos(speedAngle);
			newLink.y = racingLine[0].y - fmin(speedVector.length() * SPEED_MULTIPLIER, ADDITIONAL_DIRECTION_WAYPOINT_DISTANCE) * sin(speedAngle);
		}
		else {
			newLink.x = racingLine[0].x - ADDITIONAL_DIRECTION_WAYPOINT_DISTANCE * cos(self_ptr->getAngle());
			newLink.y = racingLine[0].y - ADDITIONAL_DIRECTION_WAYPOINT_DISTANCE * sin(self_ptr->getAngle());
		}*/
	}	

	/*doublePoint speed;
	speed.x = self_ptr->getSpeedX();
	speed.y = self_ptr->getSpeedY();

	if (speed.length() > 1 && !recoveryMode) {
		racingLine.optimizeLine(*this, atan2(speed.y, speed.x));
	}
	else*/

	racingLine.split();
	racingLine.split();

	racingLine.optimizeLine();
	while (racingLine.getCurrentPosIndex() != 0) {
		racingLine.removePoint(0);
	}
	
	result = racingLine[LOOKAHEAD_WAYPOINT_INDEX];

	minCurvature.resize(racingLine.size());
	minCurvature[0] = curvatureRadius(racingLine[0], racingLine[1], self_ptr->getAngle());
	for (int i = 1; i < minCurvature.size() - 1; i++)	{
		if (curvatureRadius(racingLine[i - 1], racingLine[i], racingLine[i + 1]) < minCurvature[i - 1]) {
			if (fabs(curvatureRadius(racingLine[i - 1], racingLine[i], racingLine[i + 1])) > 1)
				minCurvature[i] = curvatureRadius(racingLine[i - 1], racingLine[i], racingLine[i + 1]);
		}
		else {
			minCurvature[i] = minCurvature[i - 1];
		}
	}

	return result;
}

bool MyStrategy::isOnTrack(doublePoint point) const{
	bool result = false;
	int x = point.x / game_ptr->getTrackTileSize();
	int y = point.y / game_ptr->getTrackTileSize();
	if (x < 0 || y < 0 || x >= world_ptr->getWidth() || y >= world_ptr->getHeight()) {
		return false;
	}
	

	int intTileSize = game_ptr->getTrackTileSize();

	int minSafe = SAFE;
	int maxSafe = intTileSize - SAFE;

	int xInTile = (int)point.x % (int)game_ptr->getTrackTileSize();
	int yInTile = (int)point.y % (int)game_ptr->getTrackTileSize();
	
	if (xInTile > minSafe && xInTile < maxSafe && yInTile > minSafe && yInTile < maxSafe)
		return true;

	switch (world_ptr->getTilesXY()[x][y]) {
	case _UNKNOWN_TILE_TYPE_:
	case EMPTY:
		return false;
	case VERTICAL:
		return xInTile > minSafe && xInTile < maxSafe;
	case HORIZONTAL:
		return yInTile > minSafe && yInTile < maxSafe;
	case LEFT_TOP_CORNER:
		return
			(xInTile > maxSafe && (yInTile > minSafe && yInTile < maxSafe))
			||
			(yInTile > maxSafe && (xInTile > minSafe && xInTile < maxSafe));
	case RIGHT_TOP_CORNER:
		return				
			(xInTile < minSafe && (yInTile > minSafe && yInTile < maxSafe))
			||
			(yInTile > maxSafe && (xInTile > minSafe && xInTile < maxSafe));
	case LEFT_BOTTOM_CORNER:
		return
			(xInTile > maxSafe && (yInTile > minSafe && yInTile < maxSafe))
			||
			(yInTile < minSafe && (xInTile > minSafe && xInTile < maxSafe));
	case RIGHT_BOTTOM_CORNER:
		return
			(xInTile < minSafe && (yInTile > minSafe && yInTile < maxSafe))
			||
			(yInTile < minSafe && (xInTile > minSafe && xInTile < maxSafe));
	case LEFT_HEADED_T:
		return
			(xInTile > minSafe && xInTile < maxSafe)
			||
			(xInTile < minSafe && (yInTile > minSafe && yInTile < maxSafe));
	case RIGHT_HEADED_T:
		return
			(xInTile > minSafe && xInTile < maxSafe)
			||
			(xInTile > maxSafe && (yInTile > minSafe && yInTile < maxSafe));
	case TOP_HEADED_T:
		return
			(yInTile > minSafe && yInTile < maxSafe)
			||
			(yInTile < maxSafe && xInTile > minSafe && xInTile < maxSafe);
	case BOTTOM_HEADED_T:
		return
			((yInTile > minSafe && yInTile < maxSafe)
			||
			(yInTile > minSafe && xInTile > minSafe && xInTile < maxSafe));
	case CROSSROADS:
		return
			(yInTile > minSafe && xInTile > minSafe && xInTile < maxSafe)
			||
			(yInTile < maxSafe && xInTile > minSafe && xInTile < maxSafe)
			||
			(xInTile < minSafe && yInTile > minSafe && yInTile < maxSafe)
			||
			(xInTile > maxSafe && yInTile > minSafe && yInTile < maxSafe);
	default:
		return false;
	}

	return result;
}

void MyStrategy::initPrevTiles() {
	intPoint currentTile;
	currentTile.x = self_ptr->getX() / game_ptr->getTrackTileSize();
	currentTile.y = self_ptr->getY() / game_ptr->getTrackTileSize();
	intPoint prevTile = currentTile;
	previousTiles.push_back(currentTile);
	switch (world_ptr->getStartingDirection()) {
	case UP:
		prevTile.y += 1;
		break;
	case DOWN:
		prevTile.y -= 1;
		break;
	case RIGHT:
		prevTile.x -= 1;
		break;
	case LEFT:
		prevTile.x += 1;
		break;
	default:
		break;
	}
	previousTiles.insert(previousTiles.begin(), prevTile);
	while (previousTiles.size() != TILES_HISTORY_SIZE)
	{
		currentTile = previousTiles.front();
		intPoint prevTile = currentTile;

		if (canGo(currentTile, RIGHT) && pathScore[0][currentTile.x + 1][currentTile.y] > pathScore[0][currentTile.x][currentTile.y]) {
			prevTile.x++;
		}
		else if (canGo(currentTile, LEFT) && pathScore[0][currentTile.x - 1][currentTile.y] > pathScore[0][currentTile.x][currentTile.y]) {
			prevTile.x--;
		}
		else if (canGo(currentTile, UP) && pathScore[0][currentTile.x][currentTile.y - 1] > pathScore[0][currentTile.x][currentTile.y]) {
			prevTile.y--;
		}
		else if (canGo(currentTile, DOWN) && pathScore[0][currentTile.x][currentTile.y + 1] > pathScore[0][currentTile.x][currentTile.y]) {
			prevTile.y++;
		}
		previousTiles.insert(previousTiles.begin(), prevTile);
	}
}

void MyStrategy::updatePath(bool force) {
	int w = 0;

	if (!force && currentPath.size() != 0 && world_ptr->getTick() < lastPathUpdateTick + MAX_TICKS_BETWEEN_PATH_UPDATES) {
		return;
	}

	lastPathUpdateTick = world_ptr->getTick();
	w = self_ptr->getNextWaypointIndex();

	intPoint currentTile;
	currentTile.x = self_ptr->getX() / game_ptr->getTrackTileSize();
	currentTile.y = self_ptr->getY() / game_ptr->getTrackTileSize();

	vector <vector <intPoint> > possiblePaths;
	vector<int> pathBonusScore;
	vector<int> nOfCheckpointsVisited;
	vector<intPoint> tempPath;
	tempPath.reserve(PATHFINDING_BRUTEFORCE_DEPTH);
	tempPath.push_back(currentTile);
	fillPaths(tempPath, possiblePaths, w, 0);
	evaluatePaths(possiblePaths, pathBonusScore, nOfCheckpointsVisited);

	int bestScore = INT_MAX, bestIndex = 0;
	for (int i = 0; i < possiblePaths.size(); i++) {
		int pathWaypoint = (w + nOfCheckpointsVisited[i]) % world_ptr->getWaypoints().size();
		intPoint lastPathTile = possiblePaths[i].back();
		int score = - pathBonusScore[i];
		if (score < bestScore) {
			bestScore = score;
			bestIndex = i;
		}
	}

	currentPath = possiblePaths[bestIndex];
	w = (w + nOfCheckpointsVisited[bestIndex]) % world_ptr->getWaypoints().size();

#ifdef PRINT_DEBUG
	debug.beginPre();
	for (int i = 1; i < currentPath.size() - 1; i++) {
		doublePoint begin, middle, end;
		begin.x = currentPath[i - 1].x * game_ptr->getTrackTileSize() + game_ptr->getTrackTileSize() / 2;
		begin.y = currentPath[i - 1].y * game_ptr->getTrackTileSize() + game_ptr->getTrackTileSize() / 2;

		middle.x = currentPath[i].x * game_ptr->getTrackTileSize() + game_ptr->getTrackTileSize() / 2;
		middle.y = currentPath[i].y * game_ptr->getTrackTileSize() + game_ptr->getTrackTileSize() / 2;

		end.x = currentPath[i + 1].x * game_ptr->getTrackTileSize() + game_ptr->getTrackTileSize() / 2;
		end.y = currentPath[i + 1].y * game_ptr->getTrackTileSize() + game_ptr->getTrackTileSize() / 2;

		double angle = middle.angle(begin, end);

		if (fabs(fabs(angle) - PI / 2) < DIRECTION_EPS) {
			double a, b, c;
			doublePoint centre = doublePoint::middle(begin, end);
			double radius = game_ptr->getTrackTileSize() / 2;
			debug.circle(centre.x, centre.y, BONUS_PICKUP_OUTER_RADIUS, 0xFF0000);
			debug.circle(centre.x, centre.y, BONUS_PICUP_INNER_RADIUS, 0xFF0000);
		}
		else {
			if (begin.x == end.x) {
				debug.rect(middle.x - BONUS_PICKUP_WIDTH, middle.y - game_ptr->getTrackTileSize() / 2, middle.x + BONUS_PICKUP_WIDTH, middle.y + game_ptr->getTrackTileSize() / 2, 0xFF0000);
			}
			else {
				debug.rect(middle.x - game_ptr->getTrackTileSize() / 2, middle.y - BONUS_PICKUP_WIDTH, middle.x + game_ptr->getTrackTileSize() / 2, middle.y + BONUS_PICKUP_WIDTH, 0xFF0000);
			}
		}
	}
	for (int j = 1; j < currentPath.size() - 1; j++) {
		doublePoint begin, middle, end;
		begin.x = currentPath[j - 1].x;
		begin.y = currentPath[j - 1].y;

		middle.x = currentPath[j].x;
		middle.y = currentPath[j].y;

		end.x = currentPath[j + 1].x;
		end.y = currentPath[j + 1].y;

		double angle = middle.angle(begin, end);

		const vector <Bonus> &bonuses = getBonuses(currentPath[j]);
		int scoreCount = 0;
		int hpCount = 0;
		int nitroCount = 0;
		int oilCount = 0;
		int ammoCount = 0;
		for (int k = 0; k < bonuses.size(); k++) {
			doublePoint p1, p2, p3;
			p1 = doublePoint(currentPath[j - 1]) * game_ptr->getTrackTileSize() + doublePoint(game_ptr->getTrackTileSize() / 2, game_ptr->getTrackTileSize() / 2);
			p2 = doublePoint(currentPath[j]) * game_ptr->getTrackTileSize() + doublePoint(game_ptr->getTrackTileSize() / 2, game_ptr->getTrackTileSize() / 2);
			p3 = doublePoint(currentPath[j + 1]) * game_ptr->getTrackTileSize() + doublePoint(game_ptr->getTrackTileSize() / 2, game_ptr->getTrackTileSize() / 2);
			doublePoint bonusCoords(bonuses[k]);
			bool onPath = false;
			if (fabs(fabs(angle) - PI / 2) < DIRECTION_EPS) {
				double a, b, c;
				doublePoint centre = doublePoint::middle(p1, p3);
				double radius = game_ptr->getTrackTileSize() / 2;
				if ((bonusCoords.x - centre.x)*(bonusCoords.x - centre.x) + (bonusCoords.y - centre.y) * (bonusCoords.y - centre.y) < (BONUS_PICKUP_OUTER_RADIUS)*(BONUS_PICKUP_OUTER_RADIUS)
					&&
					(bonusCoords.x - centre.x)*(bonusCoords.x - centre.x) + (bonusCoords.y - centre.y) * (bonusCoords.y - centre.y) > (BONUS_PICUP_INNER_RADIUS)*(BONUS_PICUP_INNER_RADIUS)) {
					onPath = true;
				}
			}
			else {
				double a, b, c;
				a = p2.y - p1.y;
				b = p2.x - p1.x;
				c = p1.x*p2.y - p2.x*p1.y;
				double distance = fabs((a * bonusCoords.x + b * bonusCoords.y + c) / (a + b));
				if (distance <= BONUS_PICKUP_WIDTH) {
					onPath = true;
				}
			}

			if (onPath) {
				debug.fillRect(bonusCoords.x - 20, bonusCoords.y - 20, bonusCoords.x + 20, bonusCoords.y + 20, 0xFF0000);
			}
		}
	}
	debug.endPre();
#endif

	while (currentPath.size() != N_OF_LOOKAHEAD_TILES)
	{
		currentTile = currentPath.back();
		intPoint nextTile = currentTile;

		if (pathScore[w][currentTile.x][currentTile.y] == 0) {
			w = (w + 1) % nOfWaypoints;
			continue;
		}

		if (canGo(currentTile, RIGHT) && pathScore[w][currentTile.x + 1][currentTile.y] < pathScore[w][currentTile.x][currentTile.y]) {
			nextTile.x++;
		}
		else if (canGo(currentTile, LEFT) && pathScore[w][currentTile.x - 1][currentTile.y] < pathScore[w][currentTile.x][currentTile.y]) {
			nextTile.x--;
		}
		else if (canGo(currentTile, UP) && pathScore[w][currentTile.x][currentTile.y - 1] < pathScore[w][currentTile.x][currentTile.y]) {
			nextTile.y--;
		}
		else if (canGo(currentTile, DOWN) && pathScore[w][currentTile.x][currentTile.y + 1] < pathScore[w][currentTile.x][currentTile.y]) {
			nextTile.y++;
		}
		currentPath.push_back(nextTile);
	}
}

void MyStrategy::fillPaths(vector<intPoint> &path, vector<vector<intPoint> > &paths, int w, int prevWaypointIndexInPath) {
	intPoint currentTile = path.back();
	intPoint waypoint;
	waypoint.x = world_ptr->getWaypoints()[w][0];
	waypoint.y = world_ptr->getWaypoints()[w][1];
	if (currentTile == waypoint) {
		prevWaypointIndexInPath = path.size() - 1;
		w = (w + 1) % world_ptr->getWaypoints().size();
	}
	else {
		for (int i = prevWaypointIndexInPath; i < path.size() - 1; i++) {
			if (path[i] == currentTile) {
				return;
			}
		}
	}

	if (path.size() == PATHFINDING_BRUTEFORCE_DEPTH) {
		paths.push_back(path);
		return;
	}

	if (canGo(currentTile, RIGHT)) {
		intPoint nextTile = currentTile;
		nextTile.x += 1;
		path.push_back(nextTile);
		fillPaths(path, paths, w, prevWaypointIndexInPath);
		path.pop_back();
	}

	if (canGo(currentTile, LEFT)) {
		intPoint nextTile = currentTile;
		nextTile.x -= 1;
		path.push_back(nextTile);
		fillPaths(path, paths, w, prevWaypointIndexInPath);
		path.pop_back();
	}

	if (canGo(currentTile, UP)) {
		intPoint nextTile = currentTile;
		nextTile.y -= 1;
		path.push_back(nextTile);
		fillPaths(path, paths, w, prevWaypointIndexInPath);
		path.pop_back();
	}

	if (canGo(currentTile, DOWN)) {
		intPoint nextTile = currentTile;
		nextTile.y += 1;
		path.push_back(nextTile);
		fillPaths(path, paths, w, prevWaypointIndexInPath);
		path.pop_back();
	}
}

void MyStrategy::evaluatePaths(vector<vector<intPoint> > &paths, vector<int> &score, vector<int> &nOfCheckpointsVisited) {
	score.resize(paths.size());
	nOfCheckpointsVisited.resize(paths.size());
	int maxNOfCheckPointsVisited = 0;
	for (int i = 0; i < paths.size(); i++) {
		int w = self_ptr->getNextWaypointIndex();
		score[i] = 0;
		nOfCheckpointsVisited[i] = 0;
		for (int j = 1; j < paths[i].size(); j++) { 
			if (world_ptr->getWaypoints()[w][0] == paths[i][j].x
				&&
				world_ptr->getWaypoints()[w][1] == paths[i][j].y) {
				w = (w + 1) % world_ptr->getWaypoints().size();
				nOfCheckpointsVisited[i]++;
			}
			intPoint currentTile = paths[i][j];
			intPoint prevTile = paths[i][j - 1];
			score[i] += pathScore[w][prevTile.x][prevTile.y] - pathScore[w][currentTile.x][currentTile.y];

		}
		if (nOfCheckpointsVisited[i] > maxNOfCheckPointsVisited) {
			maxNOfCheckPointsVisited = nOfCheckpointsVisited[i];
		}
		score[i] += nOfCheckpointsVisited[i] * WAYPOINT_BONUS;

		/* if (world_ptr->getTick() == game_ptr->getInitialFreezeDurationTicks()) {
			bool straightStart = true;
			Direction currenDirection = world_ptr->getStartingDirection();
			int deltaX = 0, deltaY = 0;
			switch (currenDirection) {
			case UP:
				deltaY = -1;
				break;
			case DOWN:
				deltaY = 1;
				break;
			case LEFT:
				deltaX = -1;
				break;
			case RIGHT:
				deltaX = 1;
				break;
			default:
				break;
			}
			for (int j = 1; j < STRAGHT_START_LENGTH; j++) {
				if (paths[i][j].x - paths[i][j - 1].x != deltaX
					||
					paths[i][j].y - paths[i][j - 1].y != deltaY) {
					straightStart = false;
					break;
				}
			}
			if (straightStart) {
				score[i] += STRAIGHT_START_BONUS;
			}
		} */
		doublePoint nextTile;
		nextTile.x = paths[i][1].x * game_ptr->getTrackTileSize() + game_ptr->getTrackTileSize() / 2;
		nextTile.y = paths[i][1].y * game_ptr->getTrackTileSize() + game_ptr->getTrackTileSize() / 2;
		if (fabs(self_ptr->getAngleTo(nextTile.x, nextTile.y)) < PI / 18) {
			score[i] += SAME_DIRECTION_BONUS;
		}
		else {
			score[i] += SAME_DIRECTION_BONUS * cos(self_ptr->getAngleTo(nextTile.x, nextTile.y));
		}

		if (currentPath.size() != 0)
		{
			if (paths[i][0] == currentPath[0]) {
				for (int j = 1; j < 5; j++) {
					if (!(paths[i][j] == currentPath[j])) {
						score[i] -= CHANGED_MIND_PENALTY;
						break;
					}
				}
			}
			else if (paths[i][0] == currentPath[1]) {
				for (int j = 1; j < 4; j++) {
					if (!(paths[i][j] == currentPath[j + 1])) {
						score[i] -= CHANGED_MIND_PENALTY;
					}
				}
			}
		}
		vector<intPoint> path = previousTiles;
		path.pop_back();
		path.insert(path.end(), paths[i].begin(), paths[i].end());
		vector<double> angles;
		angles.resize(path.size());
		for (int j = 1; j < path.size() - 1; j++) {
			doublePoint begin, middle, end;
			begin.x = path[j - 1].x;
			begin.y = path[j - 1].y;

			middle.x = path[j].x;
			middle.y = path[j].y;

			end.x = path[j + 1].x;
			end.y = path[j + 1].y;

			angles[j] = middle.angle(begin, end);
		}
		angles.front() = 0;
		angles.back() = 0;

		for (int j = 2; j < angles.size() - 1; j++) {
			if (fabs(angles[j]) < DIRECTION_EPS) {
				score[i] -= TURNAROUND_PENALTY;
			}
			else if (fabs(fabs(angles[j - 1]) - PI) < DIRECTION_EPS &&
				fabs(fabs(angles[j]) - PI) < DIRECTION_EPS) {
				score[i] += STRAIGHT_BONUS;
			}
			else if (fabs(fabs(angles[j - 1] + angles[j])) < DIRECTION_EPS) {
				score[i] += LEFT_RIGHT_BONUS;
			}
			else if (fabs(fabs(angles[j - 1] + angles[j]) - PI) < DIRECTION_EPS) {
				score[i] -= SHARP_TURN_PENALTY;
			}
		}

		for (int j = 1; j < paths[i].size() - 1; j++) {
			doublePoint begin, middle, end;
			begin.x = paths[i][j - 1].x;
			begin.y = paths[i][j - 1].y;

			middle.x = paths[i][j].x;
			middle.y = paths[i][j].y;

			end.x = paths[i][j + 1].x;
			end.y = paths[i][j + 1].y;

			double angle = middle.angle(begin, end);

			const vector <Bonus> &bonuses = getBonuses(paths[i][j]);
			int scoreCount = 0;
			int hpCount = 0;
			int nitroCount = 0;
			int oilCount = 0;
			int ammoCount = 0;
			for (int k = 0; k < bonuses.size(); k++) {
				doublePoint p1, p2, p3;
				p1 = doublePoint(paths[i][j - 1]) * game_ptr->getTrackTileSize() + doublePoint(game_ptr->getTrackTileSize() / 2, game_ptr->getTrackTileSize() / 2);
				p2 = doublePoint(paths[i][j]) * game_ptr->getTrackTileSize() + doublePoint(game_ptr->getTrackTileSize() / 2, game_ptr->getTrackTileSize() / 2);
				p3 = doublePoint(paths[i][j + 1]) * game_ptr->getTrackTileSize() + doublePoint(game_ptr->getTrackTileSize() / 2, game_ptr->getTrackTileSize() / 2);
				doublePoint bonusCoords(bonuses[k]);
				bool onPath = false;
				if (fabs(fabs(angle) - PI / 2) < DIRECTION_EPS) {
					double a, b, c;
					doublePoint centre = doublePoint::middle(p1, p3);
					double radius = game_ptr->getTrackTileSize() / 2;
					if ((bonusCoords.x - centre.x)*(bonusCoords.x - centre.x) + (bonusCoords.y - centre.y) * (bonusCoords.y - centre.y) < (BONUS_PICKUP_OUTER_RADIUS)*(BONUS_PICKUP_OUTER_RADIUS)
						&&
						(bonusCoords.x - centre.x)*(bonusCoords.x - centre.x) + (bonusCoords.y - centre.y) * (bonusCoords.y - centre.y) > (BONUS_PICUP_INNER_RADIUS)*(BONUS_PICUP_INNER_RADIUS)) {
						onPath = true;
					}
				}
				else {
					double a, b, c;
					a = p2.y - p1.y;
					b = p2.x - p1.x;
					c = p1.x*p2.y - p2.x*p1.y;
					double distance = fabs((a * bonusCoords.x + b * bonusCoords.y + c) / (a + b));
					if (distance <= BONUS_PICKUP_WIDTH) {
						onPath = true;
					}
				}

				if (onPath) {
					switch (bonuses[k].getType()) {
					case PURE_SCORE:
						scoreCount += 1;
						break;
					case REPAIR_KIT:
						hpCount += 1;
						break;
					case AMMO_CRATE:
						ammoCount += 1;
						break;
					case NITRO_BOOST:
						nitroCount += 1;
						break;
					case OIL_CANISTER:
						oilCount += 1;
						break;
					default:
						break;
					}
				}
			}
			score[i] += scoreCount * HAS_PURE_SCORE + (self_ptr->getDurability() < 0.5 ? hpCount * HAS_HP : 0) + nitroCount * HAS_NITRO +
				ammoCount * HAS_PROJECTILE + oilCount * HAS_OIL;
		}
	}
}

void MyStrategy::updateBonuses() {
	for (int x = 0; x < bonuses.size(); x++) {
		for (int y = 0; y < bonuses[x].size(); y++) {
			bonuses[x][y].clear();
			bonusHasAssignedLink[x][y].clear();
		}
	}
	for (int i = 0; i < world_ptr->getBonuses().size(); i++) {
		int x, y;
		x = world_ptr->getBonuses()[i].getX() / game_ptr->getTrackTileSize();
		y = world_ptr->getBonuses()[i].getY() / game_ptr->getTrackTileSize();
		bonuses[x][y].push_back(world_ptr->getBonuses()[i]);
		bonusHasAssignedLink[x][y].push_back(false);
	}
}