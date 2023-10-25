package tests

import (
	"bytes"
	"context"
	"fmt"
	"strings"
	"testing"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/robot"
	robotimpl "go.viam.com/rdk/robot/impl"
	"go.viam.com/test"
)

func TestCameraServer(t *testing.T) {
	fmt.Println("Starting the tests...")
	var myRobot robot.Robot
	// put all the tests in t.Run commands
	t.Run("set up the robot", func(t *testing.T) {
		myRobot = setupViamServer(context.Background(), t)
	})
	t.Run("get images method", func(t *testing.T) {
		cam, err := camera.FromRobot(myRobot, "TheRealSense")
		test.That(t, err, test.ShouldBeNil)
		_, _, err = cam.Images(context.Background())
		test.That(t, err, test.ShouldBeNil)
	})
	t.Run("shutdown the robot", func(t *testing.T) {
		test.That(t, myRobot.Close(context.Background()), test.ShouldBeNil)
	})
}

func setupViamServer(ctx context.Context, t *testing.T) robot.Robot {
	logger := golog.NewTestLogger(t)
	moduleString := strings.TrimSpace(*modulePath)
	logger.Info("testing module at %v", moduleString)
	configString := fmt.Sprintf("{"+
		"  \"network\": {"+
		"    \"bind_address\": \"0.0.0.0:90831\","+
		"    \"insecure\": true"+
		"  },"+
		"  \"components\": ["+
		"    {"+
		"      \"name\": \"TheRealSense\","+
		"      \"model\": \"viam:camera:realsense\","+
		"      \"type\": \"camera\","+
		"      \"namespace\": \"rdk\","+
		"      \"attributes\": {"+
		"        \"sensors\": ["+
		"          \"color\","+
		"          \"depth\""+
		"        ],"+
		"        \"width_px\": 640,"+
		"        \"height_px\": 480,"+
		"        \"little_endian_depth\": true"+
		"      },"+
		"      \"depends_on\": []"+
		"    }"+
		"  ],"+
		"  \"modules\": ["+
		"    {"+
		"      \"type\": \"local\","+
		"      \"name\": \"viam_realsense\","+
		"      \"executable_path\": \"%v\""+
		"    }"+
		"  ]"+
		"}", moduleString)
	cfg, err := config.FromReader(ctx, "default.json", bytes.NewReader([]byte(configString)), logger)
	test.That(t, err, test.ShouldBeNil)
	r, err := robotimpl.RobotFromConfig(ctx, cfg, logger)
	test.That(t, err, test.ShouldBeNil)
	return r
}
