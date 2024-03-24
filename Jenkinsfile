@Library('vorausPipelineUtils')
import org.voraus.PipelineUtilities
import org.voraus.VirtualEnvironment

void main() {
    node('docker-linux && ultron') {
        checkout scm // Checkout all SCMs

        preparePipeline()

        initializeWorkspaceVariablesAndJFrogCLI()
        
        docker.withRegistry('https://artifactory.vorausrobotik.com/docker/') {

            def workspaceImage
            try {
                stage('Build Workspace Image') {
                    workspaceImage = docker.build(
                        UTILS.shell.executeCommand(command: 'uuidgen', returnStdout: true),
                        '--pull .'
                    )
                }
                insideWorkspaceContainer(workspaceImage) {
                    stage('Colcon Build') {
                        sh '''
                            . /opt/ros/humble/setup.sh
                            colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
                        '''
                    }
                    stage('Test') {
                        parallel(
                            "Colcon Test":  {
                                sh '''
                                    . /opt/ros/humble/setup.sh
                                    colcon test
                                    colcon test-result
                                '''
                            },
                            "Clang Tidy": {
                                sh '''
                                    . /opt/ros/humble/setup.sh
                                    find voraus_ros_bridge/ -regex '.*\\.cpp$' | xargs clang-tidy -p build --config-file .clang-tidy
                                '''
                            }
                        )
                        junit 'build/**/test_results/**/*.xml'
                    }
                }

                stage('Build Docker Release') {
                    docker.build("voraus-ros-bridge", "-f docker/release/Dockerfile .")
                }

                publishOptionalGitHubRelease()
            }
            catch (e) {
                PIPELINE_ERROR = e
                currentBuild.currentResult = 'FAILURE'
            }
            finally {
                executePostActions(workspaceImage)
            }
        }
    }
}

void insideWorkspaceContainer(dockerImage, Closure body) {
    dockerImage.inside(WORKSPACE_CONTAINER_ARGS) {
        body()
    }
}

/** Set all pipeline variables and load utility functions.
*
*/
void preparePipeline() {
    stage('Prepare Pipeline') {
        loadGlobalPipelineUtils()
        defineGlobalVariablesAndConstants()
    }
    return
}

/** After a call to this function, a PipelineUtilities object can be accessed via global variable `UTILS`
*
*/
void loadGlobalPipelineUtils() {
    Boolean abortPreviousBuildsOnRerun = false
    UTILS = new PipelineUtilities(this, abortPreviousBuildsOnRerun)
    UTILS.shell.executeCommand('echo Loaded PipelineUtilities')
    return
}

void defineGlobalVariablesAndConstants() {
    env.Workspace
    WORKSPACE_CONTAINER_ARGS =  """
          --cpus 6
          """
    PIPELINE_ERROR = null
    return
}

void initializeWorkspaceVariablesAndJFrogCLI() {
    stage('Initialize Workspace Variables and JFrog CLI') {
        initializeGlobalWorkspaceVariables()
        UTILS.artifactory.initJFrogCLI()
    }
    return
}

/**
* Initialize all global workspace variables needed for the Pipeline.
*/
void initializeGlobalWorkspaceVariables() {
    return
}

void publishOptionalGitHubRelease() {
    if (UTILS.git.isBuildingTag()) {
        stage('Publish GitHub Release') {
                UTILS.release.runJReleaser(command: 'release', projectVersion: UTILS.git.getTag())
        }
    }
    return
}

void executePostActions(workspaceImage) {
    stage('Post Actions') {
        insideWorkspaceContainer(workspaceImage) {
            UTILS.genericStages.run()
            UTILS.postActions.run()

            if (currentBuild.currentResult != 'SUCCESS') {
                echo "ERROR | Pipeline failed due to error: ${PIPELINE_ERROR}"
            }
        }
    }
}

main()
