#include "sharethis.h"
#include <stdio.h>

ShareThis::ShareThis(int id) {
	this->id = id;
	printf("Creating a ShareThis(%i)\n", id);
}

ShareThis::~ShareThis() {
	printf("Destroying ShareThis(%i)\n", id);
}

void ShareThis::use() {
	printf("Using ShareThis(%i)\n", id);
}
