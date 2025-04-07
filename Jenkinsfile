pipeline {
	agent {
		label 'os-ubuntu2004'
	}
	environment {
		http_proxy = credentials('proxy')
		https_proxy = credentials('proxy')
	}
	options {
		skipDefaultCheckout()
	}

	stages {
		stage('Prepare') {
			steps {
				/* Increase apt lock file timeout not to fail directly if another job is running apt too */
				sh 'echo "DPkg::Lock::Timeout \"60\";" | sudo tee -a /etc/apt/apt.conf.d/99timeout'
				/*
				 * Checkout in a subfolder, as debian packaging
				 * tools require to output on '..'.
				 */
				dir('pristine') {
					checkout scm
					sh 'sudo -E apt-get update'
					sh 'sudo -E apt-get --fix-broken install -y'
					sh 'sudo -E apt-get install linux-headers-generic dkms equivs devscripts -y' // debian package
					sh 'sudo -E apt-get install python3-ply python3-git -y' // checkpatch
					// Remove old kernels
					sh 'sudo -E apt-get autoremove -y'
				}
			}
		}
		stage('Build') {
			steps {
				dir('pristine') {
					sh 'make KVERSION=$(ls /lib/modules/)'
				}
			}
		}
		stage('Check') {
			steps {
				dir('pristine') {
					sh 'find -name "*.c" -not -name "*mod.c" -not -path "*debian*" -print0 | xargs -0 /usr/src/linux-headers-$(ls /lib/modules/)/scripts/checkpatch.pl --no-tree --max-line-length=80 --strict --ignore=LINUX_VERSION_CODE --ignore=UNDOCUMENTED_DT_STRING -f'
				}
			}
		}
		stage('Package') {
			when {
				environment name: 'BRANCH_NAME', value: 'debian'
			}
			steps {
				dir('pristine') {
					script {
						sh 'dpkg-buildpackage -us -uc -b'
						sh 'lintian || true'
					}
				}
				script {
					rtUpload (
						serverId: 'artifactory-azure',
						spec: '''{
							"files": [
								{
									"pattern": "vd55g1*.deb",
									"target": "imgswlinux-debian-local/pool/vd55g1-dkms/stable/",
									"props": "deb.distribution=stable;deb.component=main;deb.architecture=armhf;deb.architecture=arm64"
								}
							]
						}''',
						failNoOp: true
					)
				}
			}
		}
	}

	post {
		always {
			// Remove debian packaging stuff
			sh 'find . -maxdepth 1 -type f -name "vd55g1*" -exec rm -v "{}" \\;'

			// Send mail
			emailext (
				subject: '$DEFAULT_SUBJECT',
				body: '$DEFAULT_CONTENT',
				recipientProviders: [[$class: 'DevelopersRecipientProvider']],
				replyTo: '$DEFAULT_REPLYTO'
			)
		}
	}
}

