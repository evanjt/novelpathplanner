#include <stdio.h>
#include <pwd.h>
#include <unistd.h>

int main(int argc, char **argv) {
    struct passwd *info = getpwuid(getuid());
    printf("Home dir: %s\n", info->pw_dir);

    return 0;
}
