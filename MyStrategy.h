#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"

//#define PRINT_DEBUG

#ifdef PRINT_DEBUG
#include "Debug.h"
#endif

#include <list>

#include <cmath>

#define PI 3.14159265358979323846

using namespace model;
using namespace std;

struct intPoint {
	int x;
	int y;

	friend bool operator == (const intPoint &point1, const intPoint &point2) {
		return point1.x == point2.x && point1.y == point2.y;
	}
};

struct doublePoint {
	double x;
	double y;

	doublePoint() : x(0), y(0) {}
	doublePoint(double x, double y) : x(x), y(y) {};
	doublePoint(const Unit &unit) {
		x = unit.getX();
		y = unit.getY();
	}
	doublePoint(intPoint point) {
		x = point.x;
		y = point.y;
	}

	double length() { return hypot(x, y); }
	double length(const doublePoint& other) { return hypot(other.x - x, other.y - y); }
	double angle(const doublePoint& start, const doublePoint &end) {
		doublePoint vector1, vector2;
		vector1.x = start.x - x;
		vector1.y = start.y - y;
		vector2.x = end.x - x;
		vector2.y = end.y - y;

		double absoluteAngle1 = atan2(vector1.y, vector1.x);
		double absoluteAngle2 = atan2(vector2.y, vector2.x);
		double relativeAngleTo = absoluteAngle1 - absoluteAngle2;

		while (relativeAngleTo > PI) {
			relativeAngleTo -= 2.0 * PI;
		}

		while (relativeAngleTo < -PI) {
			relativeAngleTo += 2.0 * PI;
		}

		return relativeAngleTo;
	}
	
	double angle(const doublePoint &other) {
		double absoluteAngleTo = atan2(other.y - y, other.x - x);

		return absoluteAngleTo;
	}

	friend double curvatureRadius(const doublePoint &start, const doublePoint &end, double angle) {
		doublePoint vector = end - start;
		double absoluteAngle = atan2(vector.y, vector.x);
		return fabs(vector.length() / fabs(sin(absoluteAngle - angle)));
	}

	friend double curvatureRadius(doublePoint a, doublePoint b, doublePoint c) {
		double lengthA = hypot(fabs(c.x - b.x), fabs(c.y - b.y));
		double lengthB = hypot(fabs(c.x - a.x), fabs(c.y - a.y));
		double lengthC = hypot(fabs(a.x - b.x), fabs(a.y - b.y));
		double p = (lengthA + lengthB + lengthC) / 2;
		double square = sqrt(p*(p - lengthA)*(p - lengthB)*(p - lengthC));
		if (fabs(square) < 0.01 && fabs(lengthA) + fabs(lengthB) + fabs(lengthC) > 1)
			return 10000;
		double radius = lengthA * lengthB * lengthC / (4 * square);
		return radius;
	}

	void normalize() {
		x = x / length();
		y = y / length();
	}

	friend doublePoint operator + (const doublePoint &point1, const doublePoint &point2) {
		doublePoint result;
		result.x = point1.x + point2.x;
		result.y = point1.y + point2.y;
		return result;
	}

	friend doublePoint operator - (const doublePoint &point1, const doublePoint &point2) {
		doublePoint result;
		result.x = point1.x - point2.x;
		result.y = point1.y - point2.y;
		return result;
	}
	
	friend doublePoint operator * (const doublePoint &point, double other) {
		doublePoint result = point;
		result.x *= other;
		result.y *= other;
		return result;
	}

	friend doublePoint operator * (double other, const doublePoint &point) {
		return point*other;
	}

	friend doublePoint operator / (const doublePoint &point, double other) {
		doublePoint result = point;
		result.x /= other;
		result.y /= other;
		return result;
	}

	static doublePoint middle(const doublePoint& point1, const doublePoint& point2) {
		doublePoint result;
		result = (point1 + point2) / 2;
		return result;
	}
};

class MyStrategy : public Strategy {
public:
    MyStrategy();

#ifdef PRINT_DEBUG
	Debug debug;
#endif

    void move(const Car& self, const World& world, const Game& game, Move& move);

	bool isOnTrack(doublePoint point) const;
	bool isOnPath(doublePoint point) const;

	const Car &getSelf() { return *self_ptr; }
	const World &getWorld() { return *world_ptr; }
	const Game &getGame() { return *game_ptr; }

	const vector <vector <vector <Bonus> > > &getBonuses() { return bonuses; }
	const vector <Bonus> &getBonuses(int x, int y) { return bonuses[x][y]; }
	const vector <Bonus> &getBonuses(intPoint point) { return bonuses[point.x][point.y]; }

	vector <vector <vector <bool> > > &getHasAssignedLink() { return bonusHasAssignedLink; }
	vector <bool> &getHasAssignedLink(int x, int y) { return bonusHasAssignedLink[x][y]; }
	vector <bool> &getHasAssignedLink(intPoint point) { return bonusHasAssignedLink[point.x][point.y]; }

private:
	void runPathfinding();
	void recursiveFindPath(intPoint from, int w, int score);
	bool canGo(intPoint from, Direction);
	bool canShoot();
	bool spillOil();
	void updatePath(bool force);
	void initPrevTiles();
	doublePoint getNextWaypoint();
	void fillPaths(vector<intPoint> &path, vector<vector<intPoint> > &possiblePaths, int w, int prevWaypointIndexInPath);
	void evaluatePaths(vector<vector<intPoint> > &paths, vector<int> &score, vector<int> &nOfCheckpointsVisited);
	void updateBonuses();

	const Car *self_ptr;
	const World *world_ptr;
	const Game *game_ptr;
	Move *move_ptr;

	int width, height;
	int nOfWaypoints;
	doublePoint nextWaypoint;
	vector <doublePoint> posOnPrevTicks;
	vector <double> minCurvature;
	bool recoveryMode;
	int recoveryModeStartTick;
	int recoveryModeEndTick;

	vector <intPoint> currentPath;
	vector <intPoint> previousTiles;
	int lastPathUpdateTick;

	vector <vector <vector <int> > > pathScore;
	vector <vector <vector <Bonus> > > bonuses;
	vector <vector <vector <bool> > > bonusHasAssignedLink;
};

class ChainLine {

public:
	
	ChainLine(MyStrategy &strategy) : strategy(strategy), currentPosIndex(0) { }
	ChainLine(MyStrategy &strategy, const vector<intPoint> &tiles);

	void appendPoint(doublePoint point, bool fixedPoint = false, double mass = 1) {
		links.push_back(point);
		fixed.push_back(fixedPoint);
		masses.push_back(mass);
	}

	void appendPoint(double x, double y, bool fixedPoint = false, double mass = 1) {
		appendPoint(doublePoint(x, y), fixedPoint, mass);
	}

	void insertPoint(doublePoint point, int index, bool fixedPoint = false, double mass = 1) {
		links.insert(links.begin() + index, point);
		fixed.insert(fixed.begin() + index, fixedPoint);
		masses.insert(masses.begin() + index, mass);
		if (index <= currentPosIndex)
			currentPosIndex++;
	}

	void insertPoint(double x, double y, int index, bool fixedPoint = false, double mass = 1) {
		insertPoint(doublePoint(x, y), index, fixedPoint, mass);
	}

	void removePoint(int i) {
		links.erase(links.begin() + i);
		fixed.erase(fixed.begin() + i);
		masses.erase(masses.begin() + i);
		if (i < currentPosIndex)
			currentPosIndex--;
	}

	doublePoint getPoint(int index) {
		return links[index];
	}

	bool getFixed(int index) {
		return fixed[index];
	}

	void setFixed(int index, bool fixedPoint) {
		fixed[index] = fixedPoint;
	}

	doublePoint & operator[](int i) {
		return links[i];
	}

	int getCurrentPosIndex() { return currentPosIndex; }

	void addShortcutsAndUpdateMasses();
	void split();
	void optimizeLine();
	void sparse();
	int size() { return links.size(); }

private:
	vector <doublePoint> links;
	vector <bool> fixed;
	vector <double> masses;
	int currentPosIndex;

	MyStrategy &strategy;
};

#endif
