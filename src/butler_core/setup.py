from setuptools import setup

package_name = "butler_core"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Butler Dev",
    author_email="dev@example.com",
    description="Core butler robot: FSM, order manager, nav bridge.",
    license="Apache-2.0",
)
