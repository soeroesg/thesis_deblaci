FROM nerfstudio/nerfstudio:latest

WORKDIR /workspace

# Installing gn-splatter
RUN git clone https://github.com/maturk/dn-splatter
RUN cd dn-splatter/ && pip install -e .
RUN pip install git+https://github.com/nerfstudio-project/gsplat.git
