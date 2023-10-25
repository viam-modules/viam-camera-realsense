package tests

import (
	"flag"
	"fmt"
	"os"
	"strings"
	"testing"
)

var modulePath = flag.String("module", "", "the path to the intel realsense module to test. If blank, will test the module from the registry.")

func TestMain(m *testing.M) {
	fmt.Println("VIAM REALSENSE MODULE INTEGRATION TESTS")
	fmt.Println("Tests defined at https://github.com/viamrobotics/viam-camera-realsense/tree/main/integration/tests")
	flag.Parse()
	moduleString := strings.TrimSpace(*modulePath)
	if moduleString == "" {
		fmt.Println("the path to the module is a required argument e.g. $ ./realsense-integration-tests -module /path/to/module")
		os.Exit(1)
	}
	// check if module even exists
	fmt.Printf("checking if file exists at %q\n", moduleString)
	_, err := os.Stat(moduleString)
	if err != nil {
		fmt.Printf("  error: %v\n", err.Error())
		os.Exit(1)
	}
	exitVal := m.Run()
	if exitVal == 0 {
		fmt.Println("all tests succeeded!")
	}
	os.Exit(exitVal)
}
