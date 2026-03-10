#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <stdio.h>
#include <chrono>
#include <thread>
#include <cstring>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <timeapi.h>
#pragma comment(lib, "winmm.lib")
#endif

#include "sim/world.cuh"
#include "vis/sender.cuh"
#include "src/training/harness.cuh"
#include "src/training/benchmark.cuh"
#include "src/training/verify.cuh"

struct Rng {
    unsigned int state;

    RS_INLINE Rng(unsigned int seed = 0) : state(seed) {}

    RS_INLINE unsigned int next() {
        state = state * 1103515245 + 12345;
        return (state >> 16) & 0x7FFF;
    }

    RS_INLINE float rand() {
        return (float)next() / 32767.0f;
    }

    RS_INLINE float rand_axis() {
        return rand() * 2.0f - 1.0f;
    }

    RS_INLINE bool chance(float thresh) {
        return rand() < thresh;
    }
};

void reset_arena_host(Arena& arena, Rng& rng) {
    int ball_idx = arena.ball_body_idx;
    arena.world.bodies[ball_idx].collision.transform.translation = Vec3(0, 0, ball::REST_Z * physics::SCALE);
    arena.world.bodies[ball_idx].linear_vel = Vec3(rng.rand_axis(), rng.rand_axis(), rng.rand_axis()) * 1000.0f * physics::SCALE;
    arena.world.bodies[ball_idx].angular_vel = Vec3::zero();

    Vec3 spawn_positions[2] = {
        Vec3(-2048.0f, -2560.0f, car::spawn::SPAWN_Z),
        Vec3(2048.0f, -2560.0f, car::spawn::SPAWN_Z)
    };

    for (int c = 0; c < arena.num_cars; c++) {
        Car& car = arena.cars[c];
        car.state = CarState();
        car.state.pos = spawn_positions[c % 2];
        car.state.rot = Mat3::identity();
        car.state.vel = Vec3::zero();
        car.state.ang_vel = Vec3::zero();
        car.state.boost_amount = car::boost::SPAWN_AMOUNT;

        int body_idx = car.body_idx;
        arena.world.bodies[body_idx].collision.transform.translation = car.state.pos * physics::SCALE;
        arena.world.bodies[body_idx].collision.transform.matrix3 = car.state.rot;
        arena.world.bodies[body_idx].linear_vel = Vec3::zero();
        arena.world.bodies[body_idx].angular_vel = Vec3::zero();
        arena.world.bodies[body_idx].collision.state = ACTIVE;
        arena.world.bodies[body_idx].collision.flags &= ~CF_NO_RESPONSE;
    }
}

#ifdef _WIN32
static bool key_held(int vk) { return (GetAsyncKeyState(vk) & 0x8000) != 0; }

CarControls poll_kbd_controls() {
    CarControls c;
    if (key_held('W')) c.throttle += 1.0f;
    if (key_held('S')) c.throttle -= 1.0f;
    if (key_held('A')) c.steer -= 1.0f;
    if (key_held('D')) c.steer += 1.0f;
    if (key_held('Q')) c.roll -= 1.0f;
    if (key_held('E')) c.roll += 1.0f;

    c.boost     = key_held(VK_LSHIFT) || key_held(VK_LBUTTON);
    c.jump      = key_held(VK_RBUTTON);
    c.handbrake = key_held(VK_LCONTROL);

    c.yaw   = c.steer;
    c.pitch = -c.throttle;

    if (c.handbrake) {
        c.roll = c.yaw;
        c.yaw  = 0.0f;
    }

    return c.clamp();
}

bool key_just_pressed(int vk, bool& prev) {
    bool now = key_held(vk);
    bool pressed = now && !prev;
    prev = now;
    return pressed;
}
#endif

[[noreturn]] void run_visualisation(const char* mesh_folder) {
    constexpr float DT = 1.0f / 120.0f;
    constexpr int FRAME_US = static_cast<int>(DT * 1e6f);

    printf("ViteLearn Visualisation Mode\n");
    printf("============================\n\n");
    printf("Sending to RocketSimVis on port %d\n\n", vis::TARGET_PORT);

#ifdef _WIN32
    {
        STARTUPINFOA si = {}; si.cb = sizeof(si);
        PROCESS_INFORMATION pi = {};
        char cmd[] = "python C:\\ViteLearn\\references\\RocketSimVis\\src\\main.py";
        if (CreateProcessA(nullptr, cmd, nullptr, nullptr, FALSE, 0, nullptr,
                           "C:\\ViteLearn\\references\\RocketSimVis", &si, &pi)) {
            CloseHandle(pi.hThread);
            CloseHandle(pi.hProcess);
            printf("Launched RocketSimVis\n");
        }
    }
#endif

    TriangleMesh* meshes[NUM_ARENA_MESHES];
    int num_meshes = load_soccar_arena(meshes, mesh_folder);
    if (num_meshes == 0) {
        printf("Failed to load arena meshes\n");
        exit(1);
    }
    printf("Loaded %d arena meshes\n\n", num_meshes);

    Arena arena;
    arena.world.num_arena_meshes = num_meshes;
    for (int m = 0; m < num_meshes; m++)
        arena.world.arena_meshes[m] = meshes[m];
    arena.init_ball();
    arena.add_car(0);
    arena.add_car(1);

    printf("Arena initialised with %d cars\n", arena.num_cars);

    vis::StateSender sender;
    if (!sender.init()) {
        printf("Failed to initialise visualisation sender\n");
        exit(1);
    }

    Rng rng(12345);
    reset_arena_host(arena, rng);

    printf("WASD=drive RMB=jump LShift=boost LCtrl=handbrake Q/E=roll Backspace=reset 2=dribble\n");

    bool prev_backspace = false;
    bool prev_2 = false;

#ifdef _WIN32
    timeBeginPeriod(1);
#endif

    auto sim_clock = std::chrono::high_resolution_clock::now();

    while (true) {
#ifdef _WIN32
        if (key_just_pressed(VK_BACK, prev_backspace)) {
            reset_arena_host(arena, rng);
            printf("Arena reset\n");
        }
        if (key_just_pressed('2', prev_2)) {
            int bi = arena.ball_body_idx;
            Vec3 above = arena.cars[0].state.pos + Vec3(0, 0, 200.0f);
            arena.world.bodies[bi].collision.transform.translation = above * physics::SCALE;
            arena.world.bodies[bi].linear_vel = Vec3::zero();
            arena.world.bodies[bi].angular_vel = Vec3::zero();
        }

        arena.cars[0].state.controls = poll_kbd_controls();
#endif
        for (int c = 1; c < arena.num_cars; c++)
            arena.cars[c].state.controls = CarControls();

        auto now = std::chrono::high_resolution_clock::now();
        int ticks = static_cast<int>(std::chrono::duration<float>(now - sim_clock).count() / DT);
        if (ticks < 1) ticks = 1;
        if (ticks > 10) ticks = 10;

        for (int t = 0; t < ticks; t++)
            arena.step(DT);
        sim_clock += std::chrono::microseconds(static_cast<long long>(ticks) * FRAME_US);

        sender.send(arena);

        auto after = std::chrono::high_resolution_clock::now();
        auto remaining = sim_clock - after;
        if (remaining.count() > 0)
            std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::microseconds>(remaining));
    }
}

[[noreturn]] void run_training(int argc, char** argv, const char* mesh_folder) {
    int requested_envs = 1024;
    int warmup_steps = 25000;
    int eval_interval = 10000;
    int utd_ratio = 20;
    int batch_size = 2048;
    float gamma = 0.99f;
    float fixed_alpha_cont = 0.02f;
    float fixed_alpha_disc = 0.1f;
    bool do_benchmark = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--envs") == 0 && i + 1 < argc)
            requested_envs = atoi(argv[++i]);
        else if (strcmp(argv[i], "--warmup") == 0 && i + 1 < argc)
            warmup_steps = atoi(argv[++i]);
        else if (strcmp(argv[i], "--eval-interval") == 0 && i + 1 < argc)
            eval_interval = atoi(argv[++i]);
        else if (strcmp(argv[i], "--utd") == 0 && i + 1 < argc)
            utd_ratio = atoi(argv[++i]);
        else if (strcmp(argv[i], "--batch-size") == 0 && i + 1 < argc)
            batch_size = atoi(argv[++i]);
        else if (strcmp(argv[i], "--gamma") == 0 && i + 1 < argc)
            gamma = static_cast<float>(atof(argv[++i]));
        else if (strcmp(argv[i], "--fixed-alpha-cont") == 0 && i + 1 < argc)
            fixed_alpha_cont = static_cast<float>(atof(argv[++i]));
        else if (strcmp(argv[i], "--fixed-alpha-disc") == 0 && i + 1 < argc)
            fixed_alpha_disc = static_cast<float>(atof(argv[++i]));
        else if (strcmp(argv[i], "--benchmark") == 0)
            do_benchmark = true;
    }

    printf("=====================================================\n");
    printf(" ViteLearn Training Mode (SAC) [build: droq-v25]\n");
    printf("=====================================================\n\n");

    cudaDeviceProp gpu_props;
    cudaGetDeviceProperties(&gpu_props, 0);
    size_t gpu_free, gpu_total;
    cudaMemGetInfo(&gpu_free, &gpu_total);
    printf("GPU: %s (%.1f GB total, %.1f GB free)\n\n",
           gpu_props.name,
           gpu_total / (1024.0 * 1024.0 * 1024.0),
           gpu_free / (1024.0 * 1024.0 * 1024.0));

    HarnessConfig cfg;
    cfg.num_envs = requested_envs;
    cfg.warmup_steps = warmup_steps;
    cfg.eval_interval = eval_interval;
    cfg.utd_ratio = utd_ratio;
    cfg.batch_size = batch_size;
    cfg.gamma = gamma;
    cfg.fixed_alpha_cont = fixed_alpha_cont;
    cfg.fixed_alpha_disc = fixed_alpha_disc;
    cfg.mesh_folder = mesh_folder;

    if (do_benchmark) {
        constexpr size_t HEADROOM = 2ULL * 1024 * 1024 * 1024;
        size_t per_transition = REPLAY_FIELDS_PER_TRANSITION * sizeof(float);
        size_t available = gpu_free > HEADROOM ? gpu_free - HEADROOM : gpu_free / 2;
        int max_cap = static_cast<int>(available / per_transition);
        if (max_cap > 30000000) max_cap = 30000000;
        if (max_cap < 500000) max_cap = 500000;
        cfg.replay_capacity = max_cap;
    }

    Harness harness;
    harness_init(&harness, cfg);

    if (do_benchmark) {
        run_benchmark(&harness);
        harness_destroy(&harness);
        exit(0);
    }

    harness_run(&harness);
    harness_destroy(&harness);

    exit(0);
}

int main(int argc, char** argv) {
#ifdef _WIN32
    const char* default_mesh = "C://ViteLearn/collision_meshes";
#else
    const char* default_mesh = "./collision_meshes";
#endif
    const char* MESH_FOLDER = default_mesh;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--meshes") == 0 && i + 1 < argc)
            MESH_FOLDER = argv[++i];
    }

    if (argc > 1 && strcmp(argv[1], "--visualise") == 0)
        run_visualisation(MESH_FOLDER);

    if (argc > 1 && strcmp(argv[1], "--verify") == 0) {
        cublas_init();
        mirror_upload_map();
        cudaStream_t stream;
        cudaStreamCreate(&stream);
        bool passed = verify_training(MESH_FOLDER, stream);
        cudaStreamDestroy(stream);
        cublas_shutdown();
        return passed ? 0 : 1;
    }

    run_training(argc, argv, MESH_FOLDER);
}
