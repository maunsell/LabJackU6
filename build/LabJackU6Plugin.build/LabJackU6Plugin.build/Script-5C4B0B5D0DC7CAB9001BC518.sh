#!/bin/zsh
export FULL_NAME=${PRODUCT_NAME}.${WRAPPER_EXTENSION}
mkdir -p "${FINAL_INSTALL_DIR}/${FULL_NAME}"
rm -R "${FINAL_INSTALL_DIR}/${FULL_NAME}"
cp -R "${BUILT_PRODUCTS_DIR}/${FULL_NAME}" "${FINAL_INSTALL_DIR}"
