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
				sh 'git clone https://git.linuxtv.org/media_tree.git --depth 1 -b master || true' // if already cloned
				dir('media_tree') {
					sh 'git fetch --depth 1'
					sh 'git reset --hard origin/master'
					sh 'find ../pristine/ -name "*.c" -not -name "*mod.c" -not -path "*debian*" -print0 | xargs -0 scripts/checkpatch.pl --max-line-length=80 --strict --ignore=LINUX_VERSION_CODE --ignore=UNDOCUMENTED_DT_STRING -f'
				}
			}
		}
		stage('Package') {
			when { anyOf {
				environment name: 'BRANCH_NAME', value: 'master';
				environment name: 'BRANCH_NAME', value: 'debian'
			}}
			steps {
				dir('pristine') {
					dir('debianizer') {
						git credentialsId: 'af76724f-9593-4f6c-a5cc-1548eb8b0e14',
							url: 'ssh://gitolite@codex.cro.st.com/img-application-sw-linux/debianizer.git'
					}
					script {
						if (env.BRANCH_NAME == 'debian') {
							sh 'debianizer/debianizer.sh --zip'
						} else {
							sh 'debianizer/debianizer.sh --use-origin --snapshot --zip'
						}
					}
				}
			}
		}
		stage('Upload') {
			when { anyOf {
				environment name: 'BRANCH_NAME', value: 'master';
				environment name: 'BRANCH_NAME', value: 'debian'
			}}
			steps {
				script {
					if (env.BRANCH_NAME == 'debian') {
						rtUpload (
							serverId: 'artifactory-azure',
							spec: '''{
								"files": [
									{
										"pattern": "vd55g1*.deb",
										"target": "imgswlinux-debian-local/pool/vd55g1-dkms/stable/",
										"props": "deb.distribution=stable;deb.component=main;deb.architecture=armhf;deb.architecture=arm64"
									},
									{
										"pattern": "vd55g1*.zip",
										"target": "imgswlinux-releases-imgappswlinux-codex-st-com/drivers/vd55g1/stable/"
									}
								]
							}'''
						)
					} else {
						rtUpload (
							serverId: 'artifactory-azure',
							spec: '''{
								"files": [
									{
										"pattern": "vd55g1*.deb",
										"target": "imgswlinux-debian-local/pool/vd55g1-dkms/unstable/",
										"props": "deb.distribution=unstable;deb.component=main;deb.architecture=armhf;deb.architecture=arm64"
									},
									{
										"pattern": "vd55g1*.zip",
										"target": "imgswlinux-releases-imgappswlinux-codex-st-com/drivers/vd55g1/unstable/"
									}
								]
							}'''
						)
					}
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

