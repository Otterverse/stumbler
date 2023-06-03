package main

import (
	"context"
	"fmt"
	"sync"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.uber.org/zap"

	"github.com/RobinUS2/golang-moving-average"

	"github.com/viam-labs/stumbler/mpu9250"

	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"

	"go.viam.com/utils"
)

var myModel = resource.NewModel("viam-labs", "demo", "stumbler")

func main() {
	utils.ContextualMain(mainWithArgs, module.NewLoggerFromArgs("stumbler"))
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) error {
	// Instantiate the module itself
	myMod, err := module.NewModuleFromArgs(ctx, logger)
	if err != nil {
		return err
	}

	// We first put our component's constructor in the registry, then tell the module to load it
	// Note that all resources must be added before the module is started.
	resource.RegisterComponent(generic.API, myModel, resource.Registration[resource.Resource, *Config]{
		Constructor: newStumbler,
	})
	err = myMod.AddModelFromRegistry(ctx, generic.API, myModel)
	if err != nil {
		return err
	}

	err = myMod.AddModelFromRegistry(ctx, movementsensor.API, mpu9250.Model)
	if err != nil {
		return err
	}

	// The module is started.
	err = myMod.Start(ctx)
	// Close is deferred and will run automatically when this function returns.
	defer myMod.Close(ctx)
	if err != nil {
		return err
	}

	// This will block (leaving the module running) until the context is cancelled.
	// The utils.ContextualMain catches OS signals and will cancel our context for us when one is sent for shutdown/termination.
	<-ctx.Done()
	// The deferred myMod.Close() will now run.
	return nil
}


type Config struct {
	IMU string `json:"imu"`
}

// Validate ensures all parts of the config are valid.
func (cfg *Config) Validate(path string) ([]string, error) {
	if cfg.IMU == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "imu")
	}
	return []string{cfg.IMU}, nil
}

func newStumbler(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger *zap.SugaredLogger) (resource.Resource, error) {
	newStumbler := &stumbler{
		Named: conf.ResourceName().AsNamed(),
		logger: logger,
	}

	return newStumbler, newStumbler.Reconfigure(ctx, deps, conf)
}

// counter is the representation of this model. It holds only a "total" count.
type stumbler struct {
	resource.Named
	logger golog.Logger

	mu sync.RWMutex
	imu movementsensor.MovementSensor
	cancelFunc func()
	wg sync.WaitGroup
}

func (s *stumbler) Reconfigure(ctx context.Context, deps resource.Dependencies, conf resource.Config) error {
	newConf, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return err
	}

	imu, err := movementsensor.FromDependencies(deps, newConf.IMU)
	if err != nil {
		return err
	}
	s.mu.Lock()
	s.imu = imu
	s.mu.Unlock()
	return s.startBackground()
}

func (s *stumbler) DoCommand(ctx context.Context, req map[string]interface{}) (map[string]interface{}, error) {
	// We look for a map key called "command"
	cmd, ok := req["command"]
	if !ok {
		return nil, errors.New("missing 'command' string")
	}

	// If it's "get" we return the current total.
	if cmd == "get" {
		return map[string]interface{}{}, nil
	}

	// The command must've been something else.
	return nil, fmt.Errorf("unknown command string %s", cmd)
}

func (s *stumbler) Close(ctx context.Context) error {
	s.logger.Info("SMURF CLOSING STUMBLER")
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.cancelFunc != nil {
		s.cancelFunc()
	}
	s.wg.Wait()
	return nil
}

func (s *stumbler) startBackground() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.wg.Add(1)
	ctx, cancelFunc := context.WithCancel(context.Background())
	s.cancelFunc = cancelFunc
	go func(imu movementsensor.MovementSensor){
		defer s.wg.Done()
		startTime := time.Now()
		var iterations int

		X := movingaverage.New(50)
		Y := movingaverage.New(50)
		Z := movingaverage.New(50)

		RX := movingaverage.New(50)
		RY := movingaverage.New(50)
		RZ := movingaverage.New(50)

		for {
			if ctx.Err() != nil || iterations >= 10000 {
				break
			}
			// if !utils.SelectContextOrWait(ctx, time.Millisecond*1000) {
			// 	break
			// }

			accel, err := imu.LinearAcceleration(ctx, nil)
			if err != nil {
				s.logger.Error(err)
			}
			X.Add(accel.X)
			Y.Add(accel.Y)
			Z.Add(accel.Z)

			gyro, err := imu.AngularVelocity(ctx, nil)
			if err != nil {
				s.logger.Error(err)
			}
			RX.Add(gyro.X)
			RY.Add(gyro.Y)
			RZ.Add(gyro.Z)

			//s.logger.Infow("SMURF", "Acceleration", accel, "Gyro", gyro)
			iterations++
		}
		runtime := time.Now().Sub(startTime)
		updateFreq := float64(iterations) / runtime.Seconds()
		s.logger.Infow("Run Finished", "Runtime", runtime, "Iterations", iterations, "Frequency", updateFreq)
		s.logger.Infow("Averages", "X", X.Avg(), "Y", Y.Avg(), "Z", Z.Avg(), "RX", RX.Avg(), "RY", RY.Avg(), "RZ", RZ.Avg())


		pose, err := imu.Orientation(ctx, nil)
		if err != nil {
			s.logger.Error(err)
		}
		s.logger.Infow("Orientation", "pitch", pose.EulerAngles().Pitch, "roll", pose.EulerAngles().Roll, "yaw", pose.EulerAngles().Yaw)


	}(s.imu)
	return nil
}
