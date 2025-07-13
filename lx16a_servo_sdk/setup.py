from setuptools import setup

setup(
    name='lewansoul-lx16_sdk',
    version='1.0.0',
    packages=['lx16aservo_sdk'],
    url='https://github.com/zeta0707/lx16aservo_sdk',
    license='UNLICENSE',
    author='ChangWhan Lee',
    author_email='zeta0707@gmail.com',
    description='This is source code, lx16a -> dynamixle SDK',
    long_description_content_type="text/markdown",
    install_requires=['pyserial'],
    classifiers=[
        'Development Status :: 4 - Beta',
        'License :: OSI Approved :: The Unlicense (Unlicense)',
        'Operating System :: POSIX :: Other',
        'Programming Language :: Python :: 3',
        'Topic :: System :: Hardware :: Universal Serial Bus (USB) :: Communications Device Class (CDC)'
    ]
)
