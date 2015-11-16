#include "model/Player.h"

using namespace model;
using namespace std;

Player::Player()
        : id(-1), me(false), name(""), strategyCrashed(false), score(-1) { }

Player::Player(long long id, bool me, const string& name, bool strategyCrashed, int score)
        : id(id), me(me), name(name), strategyCrashed(strategyCrashed), score(score) { }

long long Player::getId() const {
    return id;
}

bool Player::isMe() const {
    return me;
}

const string& Player::getName() const {
    return name;
}

bool Player::isStrategyCrashed() const {
    return strategyCrashed;
}

int Player::getScore() const {
    return score;
}
