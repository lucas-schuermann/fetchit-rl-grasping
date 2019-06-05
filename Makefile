# directory of the main source code
PACKAGE_FOLDER=lvs-exploration

.PHONY: fix
fix:
	yapf --style google -i -r ${PACKAGE_FOLDER}

.PHONY: clean
clean:
	find . -name *.log -type f -delete
	find . -name *.dot -type f -delete
	find . -name *.pyc -delete

	find . -name *.egg-info -exec rm -rf {} \;
	find . -name dist -exec rm -rf {} \;

	rm -r `find . -name __pycache__ -type d`;
