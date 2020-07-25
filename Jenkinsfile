pipeline {
    agent any
    stages {
	stage('Preparing the test') {
	    steps {
		sh '''
		if [ -d "matlab-tests" ]; then
		    cd matlab-tests
		    git reset --hard
		    git pull
		else
		    git clone https://github.com/dqrobotics/matlab-tests.git
		    cd matlab-tests
		fi
		'''
	    }
	}

	stage('Executing tests') {
	    parallel {
		stage('Executing basic tests') {
		    steps {
			sh '''
			# The test user must define the environment variable $MATLAB_HOME
			#if [[ "$OSTYPE" == "darwin"* ]]; then
			#    source ~/.zshenv
			#else
			    # Unknown.
			#    exit 1
			#fi

			# Enter in the appropriate directory to ensure that the official test is executed.
			# By "official" we mean the test from the GitHub repository. This is to prevent test users
			# from fiddling with the test to bypass it.

			# Uncomment the two lines below to ensure that the remote code is being executed
			cd matlab-tests
			$MATLAB_HOME/matlab -nodisplay -nosplash -nodesktop -r "run(\'./run_basic_tests\')"

			#$MATLAB_HOME/matlab -nodisplay -nosplash -nodesktop -r "run(\'run_basic_tests\')"

			if [ $? -eq 0 ]; then
			    echo "Matlab test executed."
			else
			    echo "Matlab test was not executed"
			    exit 1
			fi

			'''
		    }
		}

		stage('Executing all examples') {
		    steps {
			sh '''
			# The test user must define the environment variable $MATLAB_HOME

			#if [[ "$OSTYPE" == "darwin"* ]]; then
			#    source ~/.zshenv
			#else
			    # Unknown.
			#    exit 1
			#fi

			# Enter in the appropriate directory to ensure that the official test is executed.
			# By "official" we mean the test from the GitHub repository. This is to prevent test users
			# from fiddling with the test to bypass it.

			# Uncomment the two lines below to ensure that the remote code is being executed
			cd matlab-tests
			$MATLAB_HOME/matlab -nodisplay -nosplash -nodesktop -r "run(\'./run_all_examples\')"

			# Temporary code to prevent meddling with repository
			# $MATLAB_HOME/matlab -nodisplay -nosplash -nodesktop -r "run(\'run_examples_test\')"

			if [ $? -eq 0 ]; then
			    echo "Matlab test executed."
			else
			    echo "Matlab test was not executed"
			    exit 1
			fi
			'''
		    }
		}

	    }
	}

	stage('Cleaning up the workspace') {
	    steps {
		cleanWs(cleanWhenAborted: true, cleanWhenFailure: true, cleanWhenNotBuilt: true, cleanWhenSuccess: true, cleanWhenUnstable: true, cleanupMatrixParent: true, deleteDirs: true, disableDeferredWipeout: true)
	    }
	}

    }
}
